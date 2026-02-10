#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "eterry_navigation_custom_interfaces/msg/coverage_navigation_status.hpp"
#include "eterry_vs.hpp"
#include <cv_bridge/cv_bridge.h>
#include <memory>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <chrono>
#include <fstream>

using namespace BT;

// ============================================
// CLASSE VISION NAVIGATION ACTION - CLEAN VERSION
// ============================================

class VisionNavigationAction : public StatefulActionNode
{
public:

    VisionNavigationAction(const std::string& name, const NodeConfiguration& config)
        : StatefulActionNode(name, config)
    {
        auto logger = rclcpp::get_logger("vision");
        
        // Get ROS node from blackboard
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
        
        declareAllParameters();
        
        if (!vs_controller_.readRUNParmas(ros_node_.get())) {
            throw RuntimeError("Vision parameter loading failed");
        }
        
        cmd_vel_pub_ = ros_node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        visual_status_pub_ = ros_node_->create_publisher<std_msgs::msg::Bool>("/visual_available", 10);
        visualization_pub_ = ros_node_->create_publisher<sensor_msgs::msg::Image>("/vision_navigation/visualization", 10);
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

        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(ros_node_);

        spin_thread_ = std::thread([this]() {
            executor_->spin();
        });
    }
    
    ~VisionNavigationAction() {
        if (executor_) {
            executor_->cancel();
        }
        
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
    }
    
    static PortsList providedPorts()
    {
        return { 
            InputPort<int>("row_number"),
            InputPort<double>("desired_speed"),
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
        
        // Get input values
        int row_number = 0;
        double desired_speed = 0.0;
        
        if (!getInput("row_number", row_number)) {
            row_number = 0;
        }
        
        if (!getInput("desired_speed", desired_speed)) {
            desired_speed = 0.5;
        }
        
        if (!getInput("show_visualization", show_visualization_)) {
            show_visualization_ = false;
        }
        
        RCLCPP_INFO(ros_node_->get_logger(), "Row: %d, Speed: %.2f", row_number, desired_speed);
        
        // Initialize camera pointer
        I_primary_ = (vs_controller_.camera_ID == 1) ? 
                    &vs_controller_.front_cam : 
                    &vs_controller_.back_cam;
        
        if (!I_primary_) {
            RCLCPP_ERROR(ros_node_->get_logger(), "Camera pointer is null!");
            return NodeStatus::FAILURE;
        }
        
        // Initialize neighborhood
        vs_controller_.initialize_neigbourhood(*I_primary_);
        
        publishVisualAvailable(true);
        
        // Create timer for vision loop
        control_timer_ = ros_node_->create_wall_timer(
            std::chrono::milliseconds(1000 / vs_controller_.fps),
            [this]() { 
                this->executeVisionLoop(); 
            });
        
        if (show_visualization_) {
            cv::namedWindow("Vision Navigation", cv::WINDOW_AUTOSIZE);
        }
        
        return NodeStatus::RUNNING;
    }
    
    NodeStatus onRunning() override
    {
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
        if (!is_running_) {
            return;
        }
        
        frame_counter_++;
        
        if (!I_primary_) {
            return;
        }
        
        // Check if we have an image
        if (I_primary_->image.empty()) {
            return;
        }
        
        // Check if we have points
        if (I_primary_->points.size() == 0) {
            geometry_msgs::msg::Twist stop_cmd;
            cmd_vel_pub_->publish(stop_cmd);
            return;
        }
        
        // Compute features and control
        vs_controller_.compute_feature_point(*I_primary_);
        vs_controller_.publishVisualStatus(*I_primary_);
        vs_controller_.Controller(*I_primary_);
        
        // Log velocity every 10 frames
        if (frame_counter_ % 10 == 0) {
            RCLCPP_INFO(ros_node_->get_logger(), 
                       "Vel: lin=%.3f, ang=%.3f, Points: %zu",
                       vs_controller_.VelocityMsg.linear.x,
                       vs_controller_.VelocityMsg.angular.z,
                       I_primary_->points.size());
        }
        
        // Publish command
        cmd_vel_pub_->publish(vs_controller_.VelocityMsg);
        
        // Create visualization
        try {
            // Make a copy for visualization
            cv::Mat vis_image = I_primary_->image.clone();
            
            // FIX: Draw neighborhood on the ACTUAL camera object (not a copy)
            // This draws the ROI/neighborhood boundaries
            vs_controller_.draw_neighbourhood(*I_primary_);
            
            // Draw features on the visualization image
            // We need to pass the visualization image to draw_features
            // Since draw_features expects a camera object, we create a temp one
            eterry_vs::camera temp_cam;
            temp_cam.image = vis_image;
            temp_cam.points = I_primary_->points;
            temp_cam.nh_points = I_primary_->nh_points;
            
            // Draw desired and actual features
            vs_controller_.draw_features(temp_cam, vs_controller_.F_des, cv::Scalar(0, 255, 0));
            vs_controller_.draw_features(temp_cam, vs_controller_.F, cv::Scalar(0, 0, 255));
            
            // Draw neighborhood points (orange circles)
            for(size_t i = 0; i < I_primary_->nh_points.size(); i++) {
                cv::circle(vis_image, 
                          cv::Point(I_primary_->nh_points[i].x, I_primary_->nh_points[i].y),
                          5, cv::Scalar(0, 204, 255), cv::FILLED, 8, 0);
            }
            
            // Add stats text
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
            
            // Scale if needed
            cv::Mat des_comp;
            cv::resize(vis_image, des_comp, cv::Size(), vs_controller_.Scale, vs_controller_.Scale);
            
            // Publish visualization
            publishVisualizationImage(des_comp);
            
            // Show local window if enabled
            if (show_visualization_) {
                cv::imshow("Vision Navigation", des_comp);
                cv::waitKey(1);
            }
            
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
    
    void frontImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        image_counter_++;
        
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            
            // Update camera image
            if (I_primary_) {
                I_primary_->image = cv_ptr->image;
            }
            
            // Process the image
            if (I_primary_) {
                processCameraImage(*I_primary_);
            }
            
            // Log every 20 images
            if (image_counter_ % 20 == 0) {
                RCLCPP_INFO(ros_node_->get_logger(), 
                           "Processed image %d: %zu points, %zu nh_points",
                           image_counter_,
                           vs_controller_.front_cam.points.size(),
                           vs_controller_.front_cam.nh_points.size());
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(ros_node_->get_logger(), "Image processing error: %s", e.what());
        }
    }
    
    void processCameraImage(eterry_vs::camera& cam)
    {
        cam.contours = vs_controller_.CropRowFeatures(cam);
        
        if(!vs_controller_.mask_tune || cam.contours.size() != 0){
            cam.points = vs_controller_.getContureCenters(cam.image, cam.contours);
            cam.nh_points = vs_controller_.filterContures(cam.image, cam.contours);
            
            // If filterContures returns empty, use all points
            if (cam.nh_points.size() == 0 && cam.points.size() > 0) {
                cam.nh_points = cam.points;
            }
            
            vs_controller_.is_in_neigbourhood(cam);
            
            if (cam.nh_points.size() > 0) {
                cam.lines = vs_controller_.FitLineOnContures(cam.image, cam.nh_points);
            }
            
        } else {
            geometry_msgs::msg::Twist stop_cmd;
            cmd_vel_pub_->publish(stop_cmd);
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
            
            if (show_visualization_) {
                cv::destroyWindow("Vision Navigation");
            }
            
            is_running_ = false;
        }
    }

private:
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visual_status_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr visualization_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr front_image_sub_;
    rclcpp::Subscription<
        eterry_navigation_custom_interfaces::msg::CoverageNavigationStatus>::SharedPtr nav_status_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread spin_thread_;
    
    eterry_vs::eterryVS vs_controller_;
    
    // Camera pointer
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

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<VisionNavigationAction>("VisionNavigationAction");
}