#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/path.hpp"
#include "eterry_navigation_custom_interfaces/msg/coverage_navigation_status.hpp"
#include "eterry_vs.hpp"
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <chrono>
#include <fstream>
#include <atomic>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_msgs/msg/int32.hpp" 

using namespace BT;

// ============================================
// CLASSE VISION NAVIGATION ACTION - VERSION FINALE
// (Avec support du plan de couverture et visualisation des rangées)
// ============================================

class VisionNavigationAction : public StatefulActionNode
{
public:
    VisionNavigationAction(const std::string& name, const NodeConfiguration& config)
        : StatefulActionNode(name, config)
    {
        auto logger = rclcpp::get_logger("vision");
        
        // Récupération du nœud ROS depuis le blackboard
        rclcpp::Node::SharedPtr node_ptr = nullptr;
        const std::vector<std::string> possible_names = {"node", "node_handle", "ros_node", "nh"};
        for (const auto& key_name : possible_names) {
            auto optional_node = config.blackboard->get<rclcpp::Node::SharedPtr>(key_name);
            if (optional_node) {
                node_ptr = optional_node;
                break;
            }
        }
        if (!node_ptr) {
            RCLCPP_FATAL(logger, "No node found in blackboard!");
            node_ptr = std::make_shared<rclcpp::Node>("vision_nav_temp");
        }
        ros_node_ = node_ptr;
        
        // Initialisation TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(ros_node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        declareAllParameters();
        
        if (!vs_controller_.readRUNParmas(ros_node_.get())) {
            throw RuntimeError("Vision parameter loading failed");
        }
        
        cmd_vel_pub_ = ros_node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        visual_status_pub_ = ros_node_->create_publisher<std_msgs::msg::Bool>("/visual_available", 10);
        visualization_pub_ = ros_node_->create_publisher<sensor_msgs::msg::Image>("/vision_navigation/visualization", 10);
        nh_points_pub_ = ros_node_->create_publisher<std_msgs::msg::Int32>("/vision_navigation/nh_points_count", 10);
        vs_controller_.setVisualPublisher(visual_status_pub_);
        
        front_image_sub_ = ros_node_->create_subscription<sensor_msgs::msg::Image>(
            "/front/rgb/image_raw", 10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                this->frontImageCallback(msg);
            });
        
        nav_status_sub_ = ros_node_->create_subscription<
            eterry_navigation_custom_interfaces::msg::CoverageNavigationStatus>(
            "ET01_C_N/coverage_navigation_status", 10,
            [this](const eterry_navigation_custom_interfaces::msg::CoverageNavigationStatus::SharedPtr msg) {
                this->navStatusCallback(msg);
            });
        
        coverage_plan_sub_ = ros_node_->create_subscription<nav_msgs::msg::Path>(
            "/coverage_server/coverage_plan", 10,
            [this](const nav_msgs::msg::Path::SharedPtr msg) {
                this->coveragePlanCallback(msg);
            });

        // Subscription au statut de la vision
        vision_sub_ = ros_node_->create_subscription<std_msgs::msg::Bool>(
            "/vision_available", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                vision_available_.store(msg->data);
                RCLCPP_DEBUG(ros_node_->get_logger(), "Vision status: %s", 
                           msg->data ? "AVAILABLE" : "UNAVAILABLE");
            });

        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(ros_node_);
        spin_thread_ = std::thread([this]() { executor_->spin(); });
    }
    
    ~VisionNavigationAction() {
        if (executor_) executor_->cancel();
        if (spin_thread_.joinable()) spin_thread_.join();
    }
    
    static PortsList providedPorts()
    {
        return { 
            InputPort<bool>("show_visualization")
        };
    }
    
    NodeStatus onStart() override
    {
        RCLCPP_INFO(ros_node_->get_logger(), "Starting visual navigation");
        
        is_running_ = true;
        navigation_complete_ = false;
        vision_failed_ = false;
        frame_counter_ = 0;
        image_counter_ = 0;
        

        getInput("show_visualization", show_visualization_);
        
        
        // Initialisation de la caméra
        I_primary_ = (vs_controller_.camera_ID == 1) ? 
                    &vs_controller_.front_cam : 
                    &vs_controller_.back_cam;
        if (!I_primary_) {
            RCLCPP_ERROR(ros_node_->get_logger(), "Camera pointer is null!");
            return NodeStatus::FAILURE;
        }
        
        vs_controller_.initialize_neigbourhood(*I_primary_);
        publishVisualAvailable(true);
        
        // Timer pour la boucle de vision
        int fps = vs_controller_.fps;
        if (fps <= 0) fps = 20;
        control_timer_ = ros_node_->create_wall_timer(
            std::chrono::milliseconds(1000 / fps),
            [this]() { this->executeVisionLoop(); });
        
        return NodeStatus::RUNNING;
    }
    
    NodeStatus onRunning() override
    {
        if (!vision_available_.load()) {
            RCLCPP_WARN_THROTTLE(ros_node_->get_logger(), *ros_node_->get_clock(), 5000,
                                 "🔴 VISION PERDUE - Arrêt immédiat");
            stopRobot();
            return NodeStatus::FAILURE;
        }
        
        if (navigation_complete_) {
            RCLCPP_INFO(ros_node_->get_logger(), "Visual navigation completed");
            stopRobot();
            return NodeStatus::SUCCESS;
        }
        
        if (vision_failed_) {
            RCLCPP_ERROR(ros_node_->get_logger(), "Visual navigation failed");
            stopRobot();
            return NodeStatus::FAILURE;
        }
        
        return NodeStatus::RUNNING;
    }
    
    void onHalted() override
    {
        stopRobot();
    }

private:
    struct RowSegment {
        int id;                             // numéro de la rangée (1,2,3...)
        geometry_msgs::msg::Point start;    // point de départ (monde)
        geometry_msgs::msg::Point end;      // point d'arrivée (monde)
        double a, b, c;                     // équation de la droite: a*x + b*y + c = 0
    };
    std::vector<RowSegment> rows_;

    void declareAllParameters()
    {
        ros_node_->declare_parameter<bool>("publish_cmd_vel", true);
        ros_node_->declare_parameter<bool>("publish_linear_vel", true);
        ros_node_->declare_parameter<int>("debug_level", 3);
        ros_node_->declare_parameter<int>("max_row_num", 2000000);
        ros_node_->declare_parameter<int>("fps", 20);
        ros_node_->declare_parameter<bool>("mask_tune", false);
        ros_node_->declare_parameter<bool>("single_camera_mode", false);
        ros_node_->declare_parameter<int>("maskTuneCamera", 1);
        ros_node_->declare_parameter<double>("Scale", 0.7);
        ros_node_->declare_parameter<int>("max_Hue", 80);
        ros_node_->declare_parameter<int>("min_Hue", 40);
        ros_node_->declare_parameter<int>("max_Saturation", 255);
        ros_node_->declare_parameter<int>("min_Saturation", 50);
        ros_node_->declare_parameter<int>("max_Value", 150);
        ros_node_->declare_parameter<int>("min_Value", 100);
        ros_node_->declare_parameter<double>("minContourSize", 2.0);
        ros_node_->declare_parameter<int>("LineFitting_method", 1);
        ros_node_->declare_parameter<int>("width", 640);
        ros_node_->declare_parameter<int>("height", 480);
        ros_node_->declare_parameter<int>("ex_Xc", 320);
        ros_node_->declare_parameter<int>("ex_Yc", 240);
        ros_node_->declare_parameter<int>("nh_L", 100);
        ros_node_->declare_parameter<int>("nh_H", 300);
        ros_node_->declare_parameter<int>("nh_offset", 200);
        ros_node_->declare_parameter<int>("min_points_switch", 10);
        ros_node_->declare_parameter<int>("min_frame", 30);
        ros_node_->declare_parameter<double>("coef", 55.0);
        ros_node_->declare_parameter<double>("vf_des", 0.2);
        ros_node_->declare_parameter<double>("vb_des", 0.2);
        ros_node_->declare_parameter<double>("w_max", 0.1);
        ros_node_->declare_parameter<double>("w_min", 0.01);
        ros_node_->declare_parameter<double>("z_min", 0.01);
        ros_node_->declare_parameter<double>("ty", 0.0);
        ros_node_->declare_parameter<double>("tz", 0.7);
        ros_node_->declare_parameter<double>("rho_b", -60.0);
        ros_node_->declare_parameter<double>("rho_f", -60.0);
        ros_node_->declare_parameter<double>("lambda_x_1", 10.0);
        ros_node_->declare_parameter<double>("lambda_w_1", 1.0);
        ros_node_->declare_parameter<double>("lambda_x_2", 0.0);
        ros_node_->declare_parameter<double>("lambda_w_2", 5000.0);
        ros_node_->declare_parameter<double>("lambda_x_3", 10.0);
        ros_node_->declare_parameter<double>("lambda_w_3", 1.0);
        ros_node_->declare_parameter<double>("lambda_x_4", 0.0);
        ros_node_->declare_parameter<double>("lambda_w_4", 5000.0);
        ros_node_->declare_parameter<int>("mode", 1);
        ros_node_->declare_parameter<int>("camera_ID", 1);
        ros_node_->declare_parameter<bool>("drive_forward", true);
        ros_node_->declare_parameter<bool>("turning_mode", false);
        ros_node_->declare_parameter<int>("steering_dir", 1);
        ros_node_->declare_parameter<int>("driving_dir", 1);
    }
    
    void executeVisionLoop()
    {
        if (!is_running_) return;
        
        frame_counter_++;
        
        if (!I_primary_ || I_primary_->image.empty()) {
                // 🟢 PUBLIER 0 SI PAS D'IMAGE
                auto nh_count_msg = std_msgs::msg::Int32();
                nh_count_msg.data = 0;
                nh_points_pub_->publish(nh_count_msg);
                return;
            }        
        if (I_primary_->points.size() == 0) {
            geometry_msgs::msg::Twist stop_cmd;
            cmd_vel_pub_->publish(stop_cmd);


            auto nh_count_msg = std_msgs::msg::Int32();
            nh_count_msg.data = 0;
            nh_points_pub_->publish(nh_count_msg);

            return;
        }
        
        vs_controller_.compute_feature_point(*I_primary_);
        vs_controller_.publishVisualStatus(*I_primary_);
        vs_controller_.Controller(*I_primary_);
        
        cmd_vel_pub_->publish(vs_controller_.VelocityMsg);
            
        auto nh_count_msg = std_msgs::msg::Int32();
        nh_count_msg.data = I_primary_->nh_points.size();
        nh_points_pub_->publish(nh_count_msg);
        
        try {
            cv::Mat vis_image = I_primary_->image.clone();
            
            vs_controller_.draw_neighbourhood(*I_primary_);
            
            eterry_vs::camera temp_cam;
            temp_cam.image = vis_image;
            temp_cam.points = I_primary_->points;
            temp_cam.nh_points = I_primary_->nh_points;
            
            vs_controller_.draw_features(temp_cam, vs_controller_.F_des, cv::Scalar(0, 255, 0));
            vs_controller_.draw_features(temp_cam, vs_controller_.F, cv::Scalar(0, 0, 255));
            
            for(size_t i = 0; i < I_primary_->nh_points.size(); i++) {
                cv::circle(vis_image, 
                          cv::Point(I_primary_->nh_points[i].x, I_primary_->nh_points[i].y),
                          5, cv::Scalar(0, 204, 255), cv::FILLED, 8, 0);
            }
            
            
            std::string stats_text = "Frame: " + std::to_string(frame_counter_) +
                                    " | Points: " + std::to_string(I_primary_->points.size()) +
                                    " | NH: " + std::to_string(I_primary_->nh_points.size()) +
                                    " | Lin: " + std::to_string(vs_controller_.VelocityMsg.linear.x) +
                                    " | Ang: " + std::to_string(vs_controller_.VelocityMsg.angular.z);
            cv::putText(vis_image, stats_text,
                        cv::Point(10, 30),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.6,
                        cv::Scalar(0, 255, 0),
                        2);
            
            cv::Mat des_comp;
            cv::resize(vis_image, des_comp, cv::Size(), vs_controller_.Scale, vs_controller_.Scale);
            
            publishVisualizationImage(des_comp);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(ros_node_->get_logger(), "Visualization error: %s", e.what());
        }
    }
    
    void publishVisualizationImage(const cv::Mat& image)
    {
        try {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
            msg->header.stamp = ros_node_->now();
            msg->header.frame_id = "vision_navigation";
            visualization_pub_->publish(*msg);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(ros_node_->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    // ========== FONCTIONS AJOUTÉES ==========
    
    bool getRobotPoseInMap(geometry_msgs::msg::PoseStamped& robot_pose)
    {
        try {
            geometry_msgs::msg::TransformStamped transform;
            transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            
            robot_pose.header.stamp = ros_node_->now();
            robot_pose.header.frame_id = "map";
            robot_pose.pose.position.x = transform.transform.translation.x;
            robot_pose.pose.position.y = transform.transform.translation.y;
            robot_pose.pose.position.z = transform.transform.translation.z;
            robot_pose.pose.orientation = transform.transform.rotation;
            return true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(ros_node_->get_logger(), *ros_node_->get_clock(), 5000,
                                  "TF lookup failed: %s", ex.what());
            return false;
        }
    }
    

    void coveragePlanCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        rows_.clear();
        if (msg->poses.size() < 3) {
            RCLCPP_WARN(ros_node_->get_logger(), "Chemin trop court pour extraire des rangées");
            return;
        }

        const double TURN_THRESHOLD_DEG = 30.0;
        const double TURN_THRESHOLD_RAD = TURN_THRESHOLD_DEG * M_PI / 180.0;
        
        std::vector<geometry_msgs::msg::PoseStamped> current_row_poses;
        
        for (size_t i = 1; i < msg->poses.size() - 1; ++i) {
            double dx1 = msg->poses[i].pose.position.x - msg->poses[i-1].pose.position.x;
            double dy1 = msg->poses[i].pose.position.y - msg->poses[i-1].pose.position.y;
            double dx2 = msg->poses[i+1].pose.position.x - msg->poses[i].pose.position.x;
            double dy2 = msg->poses[i+1].pose.position.y - msg->poses[i].pose.position.y;
            
            double norm1 = std::sqrt(dx1*dx1 + dy1*dy1);
            double norm2 = std::sqrt(dx2*dx2 + dy2*dy2);
            if (norm1 < 0.01 || norm2 < 0.01) continue;
            
            double dot = dx1*dx2 + dy1*dy2;
            double cos_angle = dot / (norm1 * norm2);
            cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
            double angle = std::acos(cos_angle);
            
            if (angle > TURN_THRESHOLD_RAD) {
                if (!current_row_poses.empty()) {
                    RowSegment row;
                    row.id = rows_.size() + 1;
                    row.start = current_row_poses.front().pose.position;
                    row.end = current_row_poses.back().pose.position;
                    
                    double dx = row.end.x - row.start.x;
                    double dy = row.end.y - row.start.y;
                    double norm = std::sqrt(dx*dx + dy*dy);
                    if (norm > 1e-6) {
                        row.a = dy / norm;
                        row.b = -dx / norm;
                        row.c = -(row.a * row.start.x + row.b * row.start.y);
                    } else {
                        row.a = 0; row.b = 0; row.c = 0;
                    }
                    rows_.push_back(row);
                    current_row_poses.clear();
                }
            } else {
                if (current_row_poses.empty()) {
                    current_row_poses.push_back(msg->poses[i-1]);
                }
                current_row_poses.push_back(msg->poses[i]);
            }
        }
        
        if (!current_row_poses.empty()) {
            RowSegment row;
            row.id = rows_.size() + 1;
            row.start = current_row_poses.front().pose.position;
            row.end = current_row_poses.back().pose.position;
            double dx = row.end.x - row.start.x;
            double dy = row.end.y - row.start.y;
            double norm = std::sqrt(dx*dx + dy*dy);
            if (norm > 1e-6) {
                row.a = dy / norm;
                row.b = -dx / norm;
                row.c = -(row.a * row.start.x + row.b * row.start.y);
            } else {
                row.a = 0; row.b = 0; row.c = 0;
            }
            rows_.push_back(row);
        }
        
        RCLCPP_INFO(ros_node_->get_logger(), "Plan analysé : %zu rangées détectées", rows_.size());
    }

    void frontImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        image_counter_++;
        
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            
            if (I_primary_) {
                I_primary_->image = cv_ptr->image;
                processCameraImage(*I_primary_);
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(ros_node_->get_logger(), "Image processing error: %s", e.what());
        }
    }
    
void processCameraImage(eterry_vs::camera& cam)
{
    try {
        // Étape 1: Détecter tous les contours (plantes) dans l'image
        cam.contours = vs_controller_.CropRowFeatures(cam);
        
        if (!vs_controller_.mask_tune || !cam.contours.empty()) {
            // Étape 2: Obtenir les centres de TOUS les contours
            // CES POINTS SERONT DESSINÉS EN VERT
            cam.points = vs_controller_.getContureCenters(cam.image, cam.contours);
            
            // Étape 3: Initialiser la position du voisinage
            // (basée sur les détections précédentes ou position par défaut)
            vs_controller_.initialize_neigbourhood(cam);
            
            // Étape 4: Filtrer les points pour ne garder que ceux dans le voisinage
            // CES POINTS SERONT DESSINÉS EN JAUNE
            vs_controller_.is_in_neigbourhood(cam);

            auto nh_count_msg = std_msgs::msg::Int32();
            nh_count_msg.data = cam.nh_points.size();
            nh_points_pub_->publish(nh_count_msg);
            
            // Étape 5: Si assez de points dans le voisinage, ajuster une ligne
            if (cam.nh_points.size() >= 3) {
                cam.lines = vs_controller_.FitLineOnContures(cam.image, cam.nh_points);
                
                // Étape 6: Recentrage périodique du voisinage
                static int frame_counter = 0;
                frame_counter++;
                if (frame_counter % 30 == 0) {  // Toutes les 30 frames
                    // Re-centrer le voisinage sur les points actuels
                    float sumX = 0, sumY = 0;
                    for (const auto& point : cam.nh_points) {
                        sumX += point.x;
                        sumY += point.y;
                    }
                    // Ajustement progressif
                    cam.nh.Xc = 0.7 * cam.nh.Xc + 0.3 * (sumX / cam.nh_points.size());
                    cam.nh.Yc = 0.7 * cam.nh.Yc + 0.3 * (sumY / cam.nh_points.size());
                }
            } else {
                RCLCPP_WARN_THROTTLE(ros_node_->get_logger(), 
                                    *ros_node_->get_clock(), 2000,
                                    "Pas assez de points dans le voisinage: %zu", 
                                    cam.nh_points.size());
                cam.lines.clear();
            }
        } else {
            // Aucun contour détecté
            cam.points.clear();
            cam.nh_points.clear();
            cam.lines.clear();

            auto nh_count_msg = std_msgs::msg::Int32();
            nh_count_msg.data = 0;
            nh_points_pub_->publish(nh_count_msg);

            // Arrêter le robot
            geometry_msgs::msg::Twist stop_cmd;
            cmd_vel_pub_->publish(stop_cmd);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(ros_node_->get_logger(), 
                    "Erreur dans processCameraImage: %s", e.what());
    }
}
    
    void navStatusCallback(
        const eterry_navigation_custom_interfaces::msg::CoverageNavigationStatus::SharedPtr msg)
    {
        current_nav_state_ = msg->navigation_state;
        current_row_ = msg->current_row;
        
        if (current_nav_state_ != "straight_line") {
            navigation_complete_ = true;
        }
    }
    
    void publishVisualAvailable(bool available)
    {
        auto msg = std_msgs::msg::Bool();
        msg.data = available;
        visual_status_pub_->publish(msg);
    }
    
    void stopRobot()
    {
        if (is_running_) {
            if (control_timer_) {
                control_timer_->cancel();
                control_timer_.reset();
            }
            
            geometry_msgs::msg::Twist stop_cmd;
            cmd_vel_pub_->publish(stop_cmd);
            
            publishVisualAvailable(false);
            
            is_running_ = false;
            
            RCLCPP_INFO(ros_node_->get_logger(), "🛑 Robot stopped");
        }
    }

private:
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visual_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr nh_points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visualization_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr front_image_sub_;
    rclcpp::Subscription<eterry_navigation_custom_interfaces::msg::CoverageNavigationStatus>::SharedPtr nav_status_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr coverage_plan_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread spin_thread_;
    
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vision_sub_;
    std::atomic<bool> vision_available_{true};
    
    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    eterry_vs::eterryVS vs_controller_;
    eterry_vs::camera* I_primary_ = nullptr;
    
    std::mutex image_mutex_;
    
    bool is_running_ = false;
    bool navigation_complete_ = false;
    bool vision_failed_ = false;
    bool show_visualization_ = true;
    std::string current_nav_state_;
    int current_row_ = 0;
    int frame_counter_ = 0;
    int image_counter_ = 0;
};

// ============================================
// ENREGISTREMENT DU PLUGIN
// ============================================
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<VisionNavigationAction>("VisionNavigationAction");
}