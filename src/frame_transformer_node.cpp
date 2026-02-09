#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

class FrameTransformer : public rclcpp::Node
{
public:
    FrameTransformer() : Node("frame_transformer")
    {
        // Déclaration des paramètres pour les noms de frames
        this->declare_parameter<std::string>("target_lidar_frame", "os_lidar");
        this->declare_parameter<std::string>("target_imu_frame", "os_imu");
        this->declare_parameter<std::string>("input_points_topic", "/ouster/points");
        this->declare_parameter<std::string>("input_imu_topic", "/ouster/imu");
        this->declare_parameter<std::string>("output_points_topic", "/ouster/points1");
        this->declare_parameter<std::string>("output_imu_topic", "/ouster/imu1");

        // Récupération des paramètres
        target_lidar_frame_ = this->get_parameter("target_lidar_frame").as_string();
        target_imu_frame_ = this->get_parameter("target_imu_frame").as_string();
        std::string input_points_topic = this->get_parameter("input_points_topic").as_string();
        std::string input_imu_topic = this->get_parameter("input_imu_topic").as_string();
        std::string output_points_topic = this->get_parameter("output_points_topic").as_string();
        std::string output_imu_topic = this->get_parameter("output_imu_topic").as_string();

        // Configuration QoS pour les capteurs (Best Effort, Volatile)
        rclcpp::QoS sensor_qos(10);
        sensor_qos.best_effort();  // Au lieu de reliable
        sensor_qos.durability_volatile();  // Au lieu de transient_local

        // Publishers avec QoS sensor data
        points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            output_points_topic, sensor_qos);
        
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            output_imu_topic, sensor_qos);

        // Subscribers avec la MÊME QoS (correction importante)
        points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_points_topic, sensor_qos,  // Utiliser sensor_qos ici aussi
            std::bind(&FrameTransformer::pointsCallback, this, std::placeholders::_1));
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            input_imu_topic, sensor_qos,  // Utiliser sensor_qos ici aussi
            std::bind(&FrameTransformer::imuCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Frame transformer node started");
        RCLCPP_INFO(this->get_logger(), "Transforming lidar frame to: %s", target_lidar_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Transforming imu frame to: %s", target_imu_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Using sensor data QoS (Best Effort, Volatile)");
    }

private:
    void pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Créer une nouvelle message avec le frame_id modifié
        auto transformed_msg = *msg;
        transformed_msg.header.frame_id = target_lidar_frame_;
        
        // Publier le message transformé
        points_pub_->publish(transformed_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Transformed PointCloud frame from '%s' to '%s'", 
                    msg->header.frame_id.c_str(), target_lidar_frame_.c_str());
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Créer une nouvelle message avec le frame_id modifié
        auto transformed_msg = *msg;
        transformed_msg.header.frame_id = target_imu_frame_;
        
        // Publier le message transformé
        imu_pub_->publish(transformed_msg);
        
        RCLCPP_DEBUG(this->get_logger(), "Transformed IMU frame from '%s' to '%s'", 
                    msg->header.frame_id.c_str(), target_imu_frame_.c_str());
    }

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    
    // Frame names
    std::string target_lidar_frame_;
    std::string target_imu_frame_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrameTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}