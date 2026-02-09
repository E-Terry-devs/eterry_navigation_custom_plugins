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
// CLASSE VISION NAVIGATION ACTION - DEBUG VERSION
// ============================================

class VisionNavigationAction : public StatefulActionNode
{
public:

    VisionNavigationAction(const std::string& name, const NodeConfiguration& config)
        : StatefulActionNode(name, config)
    {
        // Create debug log file
        debug_file_.open("/tmp/vision_debug.log", std::ios::out);
        debug_file_ << "===========================================\n";
        debug_file_ << "🚀 VisionNavigationAction Constructor\n";
        debug_file_ << "===========================================\n";
        
        auto logger = rclcpp::get_logger("vision");
        RCLCPP_INFO(logger, "🚀 Constructor called for %s", name.c_str());
        debug_file_ << "Constructor called for " << name << "\n";
        
        // DEBUG: List all keys in blackboard
        std::cout << "=== BLACKBOARD KEYS ===" << std::endl;
        debug_file_ << "=== BLACKBOARD KEYS ===\n";
        for (const auto& key : config.blackboard->getKeys()) {
            std::cout << "Key: " << std::string(key) << std::endl;
            debug_file_ << "Key: " << std::string(key) << "\n";
        }
        std::cout << "======================" << std::endl;
        debug_file_ << "======================\n";
        
        // Get ROS node from blackboard
        rclcpp::Node::SharedPtr node_ptr = nullptr;
        
        const std::vector<std::string> possible_names = {"node", "node_handle", "ros_node", "nh"};
        
        for (const auto& key_name : possible_names) {
            auto optional_node = config.blackboard->get<rclcpp::Node::SharedPtr>(key_name);
            if (optional_node) {
                node_ptr = optional_node;
                RCLCPP_INFO(logger, "✅ Found node with key: %s", key_name.c_str());
                debug_file_ << "Found node with key: " << key_name << "\n";
                break;
            }
        }
        
        if (!node_ptr) {
            RCLCPP_FATAL(logger, "❌ No node found in blackboard!");
            debug_file_ << "ERROR: No node found in blackboard!\n";
            node_ptr = std::make_shared<rclcpp::Node>("vision_nav_temp");
        }
        
        ros_node_ = node_ptr;
        
        RCLCPP_INFO(ros_node_->get_logger(), "===========================================");
        RCLCPP_INFO(ros_node_->get_logger(), "Creating VisionNavigationAction: %s", name.c_str());
        RCLCPP_INFO(ros_node_->get_logger(), "===========================================");
        debug_file_ << "VisionNavigationAction created: " << name << "\n";
        
        declareAllParameters();
        RCLCPP_INFO(ros_node_->get_logger(), "✅ All parameters declared");
        debug_file_ << "All parameters declared\n";
        
        RCLCPP_INFO(ros_node_->get_logger(), "📋 Loading vision controller parameters...");
        debug_file_ << "Loading vision controller parameters...\n";
        
        if (!vs_controller_.readRUNParmas(ros_node_.get())) {
            RCLCPP_ERROR(ros_node_->get_logger(), "❌ Failed to read vision parameters!");
            debug_file_ << "ERROR: Failed to read vision parameters!\n";
            throw RuntimeError("Vision parameter loading failed");
        }
        
        logLoadedParameters();
        
        RCLCPP_INFO(ros_node_->get_logger(), "📡 Creating publishers...");
        debug_file_ << "Creating publishers...\n";
        
        cmd_vel_pub_ = ros_node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        visual_status_pub_ = ros_node_->create_publisher<std_msgs::msg::Bool>("/visual_available", 10);
        vs_controller_.setVisualPublisher(visual_status_pub_);
        
        RCLCPP_INFO(ros_node_->get_logger(), "✅ Publishers created");
        debug_file_ << "Publishers created\n";
        
        RCLCPP_INFO(ros_node_->get_logger(), "📷 Creating image subscribers...");
        debug_file_ << "Creating image subscribers...\n";
        
        rclcpp::QoS image_qos(10);
        image_qos.reliable();
        
        front_image_sub_ = ros_node_->create_subscription<sensor_msgs::msg::Image>(
            "/front/rgb/image_raw", image_qos,
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

        // Start spinning in a separate thread
        spin_thread_ = std::thread([this]() {
            RCLCPP_INFO(ros_node_->get_logger(), "🔄 Starting ROS spin thread...");
            executor_->spin();
        });
        
        RCLCPP_INFO(ros_node_->get_logger(), "✅ Subscribers created");
        debug_file_ << "Subscribers created\n";
        
        RCLCPP_INFO(ros_node_->get_logger(), "===========================================");
        RCLCPP_INFO(ros_node_->get_logger(), "✅ VisionNavigationAction initialized");
        RCLCPP_INFO(ros_node_->get_logger(), "===========================================");
        debug_file_ << "VisionNavigationAction initialized\n";
        debug_file_ << "===========================================\n";
        debug_file_.flush();
    }
    
    ~VisionNavigationAction() {
        // Stop the spin thread
        if (executor_) {
            executor_->cancel();
        }
        
        if (spin_thread_.joinable()) {
            spin_thread_.join();
            RCLCPP_INFO(ros_node_->get_logger(), "🛑 ROS spin thread stopped");
        }
        
        if (debug_file_.is_open()) {
            debug_file_ << "===========================================\n";
            debug_file_ << "🧹 Destructor called\n";
            debug_file_ << "===========================================\n";
            debug_file_.close();
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
        RCLCPP_INFO(ros_node_->get_logger(), "===========================================");
        RCLCPP_INFO(ros_node_->get_logger(), "🚀 STARTING VISUAL NAVIGATION");
        RCLCPP_INFO(ros_node_->get_logger(), "===========================================");
        debug_file_ << "===========================================\n";
        debug_file_ << "🚀 STARTING VISUAL NAVIGATION\n";
        debug_file_ << "===========================================\n";
        
        is_running_ = true;
        navigation_complete_ = false;
        vision_failed_ = false;
        frame_counter_ = 0;
        last_execution_time_ = ros_node_->now();
        last_debug_time_ = ros_node_->now();
        
        // Get input values
        int row_number = 0;
        double desired_speed = 0.0;
        
        if (!getInput("row_number", row_number)) {
            RCLCPP_WARN(ros_node_->get_logger(), "No row_number provided, using default 0");
            debug_file_ << "WARN: No row_number provided, using default 0\n";
        }
        
        if (!getInput("desired_speed", desired_speed)) {
            RCLCPP_WARN(ros_node_->get_logger(), "No desired_speed provided, using default 0.5");
            debug_file_ << "WARN: No desired_speed provided, using default 0.5\n";
            desired_speed = 0.5;
        }
        
        if (!getInput("show_visualization", show_visualization_)) {
            show_visualization_ = true;
        }
        
        RCLCPP_INFO(ros_node_->get_logger(), "📊 Row number: %d", row_number);
        RCLCPP_INFO(ros_node_->get_logger(), "📊 Desired speed: %.2f", desired_speed);
        RCLCPP_INFO(ros_node_->get_logger(), "👁️  Visualization: %s", 
                   show_visualization_ ? "ENABLED" : "DISABLED");
        
        debug_file_ << "Row number: " << row_number << "\n";
        debug_file_ << "Desired speed: " << desired_speed << "\n";
        debug_file_ << "Visualization: " << (show_visualization_ ? "ENABLED" : "DISABLED") << "\n";
        
        RCLCPP_INFO(ros_node_->get_logger(), "📷 Initializing camera neighbourhood...");
        debug_file_ << "Initializing camera neighbourhood...\n";
        
        eterry_vs::camera* I_primary = (vs_controller_.camera_ID == 1) ? 
                                      &vs_controller_.front_cam : 
                                      &vs_controller_.back_cam;
        
        if (I_primary) {
            vs_controller_.initialize_neigbourhood(*I_primary);
            RCLCPP_INFO(ros_node_->get_logger(), "✅ Neighbourhood initialized for camera %d", 
                       vs_controller_.camera_ID);
            debug_file_ << "Neighbourhood initialized for camera " << vs_controller_.camera_ID << "\n";
        } else {
            RCLCPP_ERROR(ros_node_->get_logger(), "❌ Camera pointer is null!");
            debug_file_ << "ERROR: Camera pointer is null!\n";
            return NodeStatus::FAILURE;
        }
        
        publishVisualAvailable(true);
        
        control_timer_ = ros_node_->create_wall_timer(
            std::chrono::milliseconds(1000 / vs_controller_.fps),
            [this]() { this->executeVisionLoop(); });
        
        RCLCPP_INFO(ros_node_->get_logger(), "⏱️ Timer started at %d Hz", vs_controller_.fps);
        debug_file_ << "Timer started at " << vs_controller_.fps << " Hz\n";
        
        if (show_visualization_) {
            cv::namedWindow("Vision Navigation", cv::WINDOW_AUTOSIZE);
            RCLCPP_INFO(ros_node_->get_logger(), "🖥️  Visualization window created");
            debug_file_ << "Visualization window created\n";
        }
        
        debug_file_ << "===========================================\n";
        debug_file_ << "✅ onStart() completed\n";
        debug_file_ << "===========================================\n";
        debug_file_.flush();
        
        return NodeStatus::RUNNING;
    }
    
    NodeStatus onRunning() override
    {
        auto now = ros_node_->now();
        
        if ((now - last_debug_time_).seconds() > 2.0) {
            RCLCPP_INFO(ros_node_->get_logger(), 
                       "🔄 Vision Navigation Running... Frame: %d, Running: %s",
                       frame_counter_, is_running_ ? "true" : "false");
            debug_file_ << "onRunning check - Frame: " << frame_counter_ 
                       << ", is_running: " << is_running_ << "\n";
            last_debug_time_ = now;
        }
        
        if (navigation_complete_) {
            RCLCPP_INFO(ros_node_->get_logger(), "===========================================");
            RCLCPP_INFO(ros_node_->get_logger(), "✅ VISUAL NAVIGATION COMPLETED");
            RCLCPP_INFO(ros_node_->get_logger(), "===========================================");
            debug_file_ << "===========================================\n";
            debug_file_ << "✅ VISUAL NAVIGATION COMPLETED\n";
            debug_file_ << "===========================================\n";
            debug_file_.flush();
            stopRobot();
            return NodeStatus::SUCCESS;
        }
        
        if (vision_failed_) {
            RCLCPP_ERROR(ros_node_->get_logger(), "===========================================");
            RCLCPP_ERROR(ros_node_->get_logger(), "❌ VISUAL NAVIGATION FAILED");
            RCLCPP_ERROR(ros_node_->get_logger(), "===========================================");
            debug_file_ << "===========================================\n";
            debug_file_ << "❌ VISUAL NAVIGATION FAILED\n";
            debug_file_ << "===========================================\n";
            debug_file_.flush();
            stopRobot();
            return NodeStatus::FAILURE;
        }
        
        return NodeStatus::RUNNING;
    }
    
    void onHalted() override
    {
        RCLCPP_INFO(ros_node_->get_logger(), "⏸️ Visual navigation halted");
        debug_file_ << "onHalted() called\n";
        stopRobot();
    }

private:
    // ==================== HELPER METHODS ====================
    
    void declareAllParameters()
    {
        RCLCPP_INFO(ros_node_->get_logger(), "📝 Declaring all parameters...");
        debug_file_ << "Declaring all parameters...\n";
        
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
        
        RCLCPP_INFO(ros_node_->get_logger(), "✅ All parameters declared successfully");
        debug_file_ << "All parameters declared successfully\n";
    }
    
    void logLoadedParameters()
    {
        RCLCPP_INFO(ros_node_->get_logger(), "🔍 VERIFYING LOADED PARAMETERS:");
        debug_file_ << "VERIFYING LOADED PARAMETERS:\n";
        
        try {
            double scale = ros_node_->get_parameter("Scale").as_double();
            int fps = ros_node_->get_parameter("fps").as_int();
            int camera_id = ros_node_->get_parameter("camera_ID").as_int();
            double vf_des = ros_node_->get_parameter("vf_des").as_double();
            double coef = ros_node_->get_parameter("coef").as_double();
            int min_frame = ros_node_->get_parameter("min_frame").as_int();
            
            RCLCPP_INFO(ros_node_->get_logger(), "   📊 Scale: %.2f (expected: 0.7)", scale);
            RCLCPP_INFO(ros_node_->get_logger(), "   ⏱️  FPS: %d (expected: 20)", fps);
            RCLCPP_INFO(ros_node_->get_logger(), "   📷 Camera ID: %d (expected: 1)", camera_id);
            RCLCPP_INFO(ros_node_->get_logger(), "   🚀 vf_des: %.2f (expected: 0.2)", vf_des);
            RCLCPP_INFO(ros_node_->get_logger(), "   📐 coef: %.2f (expected: 55.0)", coef);
            RCLCPP_INFO(ros_node_->get_logger(), "   🖼️  min_frame: %d (expected: 30)", min_frame);
            
            debug_file_ << "   Scale: " << scale << " (expected: 0.7)\n";
            debug_file_ << "   FPS: " << fps << " (expected: 20)\n";
            debug_file_ << "   Camera ID: " << camera_id << " (expected: 1)\n";
            debug_file_ << "   vf_des: " << vf_des << " (expected: 0.2)\n";
            debug_file_ << "   coef: " << coef << " (expected: 55.0)\n";
            debug_file_ << "   min_frame: " << min_frame << " (expected: 30)\n";
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(ros_node_->get_logger(), "❌ Error reading parameters: %s", e.what());
            debug_file_ << "ERROR reading parameters: " << e.what() << "\n";
        }
    }
    
    void executeVisionLoop()
    {
        if (!is_running_) {
            debug_file_ << "executeVisionLoop: is_running_ = false\n";
            debug_file_.flush();
            return;
        }
        
        frame_counter_++;
        
        debug_file_ << "\n===========================================\n";
        debug_file_ << "🔄 executeVisionLoop - Frame: " << frame_counter_ << "\n";
        debug_file_ << "===========================================\n";
        
        RCLCPP_INFO(ros_node_->get_logger(), "🎬 Vision loop frame %d", frame_counter_);
        
        eterry_vs::camera* I_primary = nullptr;
        
        try {
            // Determine primary camera
            I_primary =   &vs_controller_.front_cam ;


            if (!I_primary) {
                RCLCPP_ERROR(ros_node_->get_logger(), "❌ Camera pointer is null!");
                debug_file_ << "ERROR: Camera pointer is null!\n";
                debug_file_.flush();
                return;
            }
            
            // Check image and points
            bool has_image = !I_primary->image.empty();
            bool has_points = I_primary->points.size() > 0;
            
            RCLCPP_INFO(ros_node_->get_logger(), 
                       "📊 Camera %d - Image: %s, Points: %zu, Contours: %zu",
                       vs_controller_.camera_ID,
                       has_image ? "YES" : "NO",
                       I_primary->points.size(),
                       I_primary->contours.size());
            
            debug_file_ << "Camera " << vs_controller_.camera_ID 
                       << " - Image: " << (has_image ? "YES" : "NO")
                       << ", Points: " << I_primary->points.size()
                       << ", Contours: " << I_primary->contours.size() << "\n";
            
            if (!has_image || !has_points) {
                RCLCPP_WARN(ros_node_->get_logger(), 
                           "⚠️ No image or points - sending STOP");
                debug_file_ << "WARN: No image or points - sending STOP\n";
                
                geometry_msgs::msg::Twist stop_cmd;
                cmd_vel_pub_->publish(stop_cmd);
                
                RCLCPP_INFO(ros_node_->get_logger(), "🛑 Published STOP command");
                debug_file_ << "Published STOP command\n";
                debug_file_.flush();
                return;
            }
            
            // Process vision
            RCLCPP_INFO(ros_node_->get_logger(), "🔍 Computing features...");
            debug_file_ << "Computing features...\n";
            
            vs_controller_.compute_feature_point(*I_primary);
            vs_controller_.publishVisualStatus(*I_primary);
            
            RCLCPP_INFO(ros_node_->get_logger(), "🎮 Executing controller...");
            debug_file_ << "Executing controller...\n";
            
            // Save controller state before execution
            debug_file_ << "Before Controller() - ";
            debug_file_ << "F size: " << vs_controller_.F.size();
            debug_file_ << ", F_des size: " << vs_controller_.F_des.size() << "\n";
            
            vs_controller_.Controller(*I_primary);
            
            // Log velocity command
            RCLCPP_INFO(ros_node_->get_logger(), 
                       "🎯 Velocity command: lin=%.3f, ang=%.3f",
                       vs_controller_.VelocityMsg.linear.x,
                       vs_controller_.VelocityMsg.angular.z);
            
            debug_file_ << "Velocity command: linear.x=" << vs_controller_.VelocityMsg.linear.x
                       << ", angular.z=" << vs_controller_.VelocityMsg.angular.z << "\n";
            
            // Check if command is non-zero
            if (abs(vs_controller_.VelocityMsg.linear.x) < 0.001 && 
                abs(vs_controller_.VelocityMsg.angular.z) < 0.001) {
                RCLCPP_WARN(ros_node_->get_logger(), "⚠️ Controller output is near zero!");
                debug_file_ << "WARN: Controller output is near zero!\n";
            }
            
            // Publish command
            cmd_vel_pub_->publish(vs_controller_.VelocityMsg);
            RCLCPP_INFO(ros_node_->get_logger(), "📤 Published cmd_vel");
            debug_file_ << "Published cmd_vel\n";
            
            // Save command to file for debugging
            std::ofstream cmd_file("/tmp/cmd_vel_debug.log", std::ios::app);
            if (cmd_file.is_open()) {
                auto now = ros_node_->now();
                cmd_file << "Frame " << frame_counter_ 
                        << " - Time: " << now.seconds() 
                        << " - lin: " << vs_controller_.VelocityMsg.linear.x
                        << " - ang: " << vs_controller_.VelocityMsg.angular.z
                        << " - Points: " << I_primary->points.size() << "\n";
                cmd_file.close();
            }
            
            // Visualization
            if(!I_primary->image.empty()) {
                try {
                    vs_controller_.draw_neighbourhood(*I_primary);
                    vs_controller_.draw_features(*I_primary, vs_controller_.F_des, cv::Scalar(0, 255, 0));
                    vs_controller_.draw_features(*I_primary, vs_controller_.F, cv::Scalar(0, 0, 255));
                    
                    for(size_t i = 0; i < I_primary->nh_points.size(); i++) {
                        cv::circle(I_primary->image, 
                                  cv::Point(I_primary->nh_points[i].x, I_primary->nh_points[i].y),
                                  5, cv::Scalar(0, 204, 255), cv::FILLED, 8, 0);
                    }
                    
                    cv::Mat des_comp;
                    cv::resize(I_primary->image, des_comp, cv::Size(), vs_controller_.Scale, vs_controller_.Scale);
                    
                    if (show_visualization_) {
                        cv::imshow("Vision Navigation", des_comp);
                        cv::waitKey(1);
                    }
                    
                    // Save debug image every 10 frames
                    if (frame_counter_ % 10 == 0) {
                        std::string filename = "/tmp/vision_frame_" + 
                                              std::to_string(frame_counter_) + ".jpg";
                        cv::imwrite(filename, des_comp);
                        RCLCPP_INFO(ros_node_->get_logger(), "💾 Saved debug frame to %s", filename.c_str());
                        debug_file_ << "Saved debug frame to " << filename << "\n";
                    }
                    
                } catch (const cv::Exception& e) {
                    RCLCPP_ERROR(ros_node_->get_logger(), "❌ OpenCV error: %s", e.what());
                    debug_file_ << "OpenCV error: " << e.what() << "\n";
                }
            }
            
            last_execution_time_ = ros_node_->now();
            
            RCLCPP_INFO(ros_node_->get_logger(), 
                       "[STAT] Frame: %d, Points: %zu, NH: %zu, Lines: %zu", 
                       frame_counter_, 
                       I_primary->points.size(),
                       I_primary->nh_points.size(),
                       I_primary->lines.size());
            
            debug_file_ << "STAT - Frame: " << frame_counter_
                       << ", Points: " << I_primary->points.size()
                       << ", NH: " << I_primary->nh_points.size()
                       << ", Lines: " << I_primary->lines.size() << "\n";
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(ros_node_->get_logger(), "❌ Error in vision control loop: %s", e.what());
            debug_file_ << "ERROR in vision control loop: " << e.what() << "\n";
            vision_failed_ = true;
            stopRobot();
        }
        
        debug_file_ << "===========================================\n";
        debug_file_ << "✅ executeVisionLoop completed\n";
        debug_file_ << "===========================================\n";
        debug_file_.flush();
    }
    
    void frontImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        static int image_counter = 0;
        image_counter++;
        
        debug_file_ << "\n===========================================\n";
        debug_file_ << "📷 frontImageCallback - Count: " << image_counter << "\n";
        debug_file_ << "===========================================\n";
        
        try {
            RCLCPP_INFO(ros_node_->get_logger(), 
                       "📷 Front image %d received: %dx%d, encoding: %s",
                       image_counter, msg->width, msg->height, msg->encoding.c_str());
            
            debug_file_ << "Image " << image_counter 
                       << " - Size: " << msg->width << "x" << msg->height
                       << ", Encoding: " << msg->encoding << "\n";
            
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            
            // Save raw image for debugging (first 5 frames)
            if (image_counter <= 5) {
                std::string filename = "/tmp/raw_front_" + 
                                      std::to_string(image_counter) + ".jpg";
                cv::imwrite(filename, cv_ptr->image);
                RCLCPP_INFO(ros_node_->get_logger(), "💾 Saved raw image to %s", filename.c_str());
                debug_file_ << "Saved raw image to " << filename << "\n";
            }
            
            vs_controller_.front_cam.image = cv_ptr->image;
            
            RCLCPP_INFO(ros_node_->get_logger(), "🖼️ Processing image...");
            debug_file_ << "Processing image...\n";
            
            processCameraImage(vs_controller_.front_cam);
            
            std::lock_guard<std::mutex> lock(image_mutex_);
            latest_front_results_ = vs_controller_.front_cam;
            
            RCLCPP_INFO(ros_node_->get_logger(),
                       "✅ Processed: %zu points, %zu nh_points, %zu contours",
                       vs_controller_.front_cam.points.size(),
                       vs_controller_.front_cam.nh_points.size(),
                       vs_controller_.front_cam.contours.size());
            
            debug_file_ << "Processed - Points: " << vs_controller_.front_cam.points.size()
                       << ", NH Points: " << vs_controller_.front_cam.nh_points.size()
                       << ", Contours: " << vs_controller_.front_cam.contours.size() << "\n";
            
            std::string str;
            std::stringstream stream, stream1, stream2;
            
            stream << vs_controller_.front_cam.points.size(); 
            stream1 << vs_controller_.front_cam.nh_points.size(); 
            stream2 << vs_controller_.camera_ID;
            
            str = "Points: " + stream.str() + 
                  " NH: " + stream1.str() + 
                  " Cam: " + stream2.str();
            
            cv::putText(vs_controller_.front_cam.image, str,
                        cv::Point(40, 40),
                        cv::FONT_HERSHEY_COMPLEX_SMALL,
                        0.8,
                        cv::Scalar(0, 255, 0),
                        2);
            
            // Save processed image for debugging
            if (image_counter % 20 == 0) {
                std::string filename = "/tmp/processed_front_" + 
                                      std::to_string(image_counter) + ".jpg";
                cv::imwrite(filename, vs_controller_.front_cam.image);
                debug_file_ << "Saved processed image to " << filename << "\n";
            }
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(ros_node_->get_logger(), "cv_bridge exception: %s", e.what());
            debug_file_ << "cv_bridge exception: " << e.what() << "\n";
        } catch (const std::exception& e) {
            RCLCPP_ERROR(ros_node_->get_logger(), "Exception: %s", e.what());
            debug_file_ << "Exception: " << e.what() << "\n";
        }
        
        debug_file_ << "===========================================\n";
        debug_file_ << "✅ frontImageCallback completed\n";
        debug_file_ << "===========================================\n";
        debug_file_.flush();
    }
    
    void processCameraImage(eterry_vs::camera& cam)
    {
        debug_file_ << "processCameraImage - Input image: " 
                   << cam.image.cols << "x" << cam.image.rows << "\n";
        
        cam.contours = vs_controller_.CropRowFeatures(cam);
        
        debug_file_ << "After CropRowFeatures - Contours: " << cam.contours.size() << "\n";
        
        if(!vs_controller_.mask_tune || cam.contours.size() != 0){
            cam.points = vs_controller_.getContureCenters(cam.image, cam.contours);
            debug_file_ << "After getContureCenters - Points: " << cam.points.size() << "\n";
            
            cam.nh_points = vs_controller_.filterContures(cam.image, cam.contours);
            debug_file_ << "After filterContures - NH Points: " << cam.nh_points.size() << "\n";
            
            vs_controller_.is_in_neigbourhood(cam);
            debug_file_ << "After is_in_neigbourhood\n";
            
            cam.lines = vs_controller_.FitLineOnContures(cam.image, cam.nh_points);
            debug_file_ << "After FitLineOnContures - Lines: " << cam.lines.size() << "\n";
            
            // Save intermediate results for debugging
            if (cam.contours.size() > 0) {
                cv::Mat contour_img = cam.image.clone();
                cv::drawContours(contour_img, cam.contours, -1, cv::Scalar(0, 255, 0), 2);
                
                std::string filename = "/tmp/contours_" + 
                                      std::to_string(contour_counter_++) + ".jpg";
                cv::imwrite(filename, contour_img);
                debug_file_ << "Saved contours image to " << filename << "\n";
            }
            
        } else {
            RCLCPP_WARN(ros_node_->get_logger(), 
                       "❌ No contours found - mask_tune=%d", vs_controller_.mask_tune);
            debug_file_ << "WARN: No contours found - mask_tune=" << vs_controller_.mask_tune << "\n";
            geometry_msgs::msg::Twist stop_cmd;
            cmd_vel_pub_->publish(stop_cmd);
        }
    }
    
    void navStatusCallback(
        const eterry_navigation_custom_interfaces::msg::CoverageNavigationStatus::SharedPtr msg)
    {
        current_nav_state_ = msg->navigation_state;
        current_row_ = msg->current_row;
        
        debug_file_ << "navStatusCallback - State: " << current_nav_state_
                   << ", Row: " << current_row_ << "\n";
        
        if (current_nav_state_ != "straight_line") {
            navigation_complete_ = true;
            RCLCPP_INFO(ros_node_->get_logger(), 
                       "📊 Navigation state changed to: %s - Completing vision navigation", 
                       current_nav_state_.c_str());
            debug_file_ << "Navigation complete - State: " << current_nav_state_ << "\n";
        }
        
        if (frame_counter_ % 20 == 0) {
            RCLCPP_INFO(ros_node_->get_logger(), 
                       "📊 Nav Status: %s, Row: %d", 
                       current_nav_state_.c_str(), current_row_);
        }
    }
    
    void publishVisualAvailable(bool available)
    {
        auto msg = std_msgs::msg::Bool();
        msg.data = available;
        visual_status_pub_->publish(msg);
        RCLCPP_INFO(ros_node_->get_logger(), "📡 Visual available published: %s", 
                   available ? "true" : "false");
        debug_file_ << "Visual available published: " << (available ? "true" : "false") << "\n";
    }
    
    void stopRobot()
    {
        if (is_running_) {
            RCLCPP_INFO(ros_node_->get_logger(), "🛑 Stopping robot...");
            debug_file_ << "stopRobot() called\n";
            
            if (control_timer_) {
                control_timer_->cancel();
                control_timer_.reset();
                RCLCPP_INFO(ros_node_->get_logger(), "⏹️ Timer stopped");
                debug_file_ << "Timer stopped\n";
            }
            
            geometry_msgs::msg::Twist stop_cmd;
            cmd_vel_pub_->publish(stop_cmd);
            RCLCPP_INFO(ros_node_->get_logger(), "📤 Stop command published");
            debug_file_ << "Stop command published\n";
            
            publishVisualAvailable(false);
            
            if (show_visualization_) {
                cv::destroyWindow("Vision Navigation");
                RCLCPP_INFO(ros_node_->get_logger(), "🖥️  Visualization window closed");
                debug_file_ << "Visualization window closed\n";
            }
            
            is_running_ = false;
            RCLCPP_INFO(ros_node_->get_logger(), "✅ Robot stopped successfully");
            debug_file_ << "Robot stopped successfully\n";
            
            debug_file_.flush();
        }
    }

private:
    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr visual_status_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr front_image_sub_;
    rclcpp::Subscription<
        eterry_navigation_custom_interfaces::msg::CoverageNavigationStatus>::SharedPtr nav_status_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread spin_thread_;
    
    eterry_vs::eterryVS vs_controller_;
    
    std::mutex image_mutex_;
    eterry_vs::camera latest_front_results_;
    
    bool is_running_ = false;
    bool navigation_complete_ = false;
    bool vision_failed_ = false;
    bool show_visualization_ = true;
    std::string current_nav_state_;
    int current_row_ = 0;
    int frame_counter_ = 0;
    int contour_counter_ = 0;
    
    rclcpp::Time last_execution_time_;
    rclcpp::Time last_debug_time_;
    
    std::ofstream debug_file_;
};

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<VisionNavigationAction>("VisionNavigationAction");
}