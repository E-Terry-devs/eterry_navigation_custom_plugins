#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <mutex>
#include <tinyxml2.h>
#include <sstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <limits>

struct Point {
    double x;
    double y;
};

struct Row {
    int id;
    Point start;
    Point end;
};

namespace Geometry {
    bool isPointInBox(const Point& point, const Point& corner1, const Point& corner2, double tolerance) {
        double minX = std::min(corner1.x, corner2.x) - tolerance;
        double maxX = std::max(corner1.x, corner2.x) + tolerance;
        double minY = std::min(corner1.y, corner2.y) - tolerance;
        double maxY = std::max(corner1.y, corner2.y) + tolerance;
        
        return (point.x >= minX && point.x <= maxX && point.y >= minY && point.y <= maxY);
    }
}

Point parsePoint(const std::string& coordStr) {
    size_t commaPos = coordStr.find(',');
    if (commaPos == std::string::npos) return {0.0, 0.0};
    double x = std::stod(coordStr.substr(0, commaPos));
    double y = std::stod(coordStr.substr(commaPos + 1));
    return {x, y};
}

// Fonction pour trouver le point de sortie vers le start de la première rangée
Point getExitPointTowardsFirstRowStart(const Row& currentRow, const Point& firstRowStart, double extra_distance = 4.0) {
    // Déterminer quel point de la rangée courante est le plus proche du start de la première rangée
    double distToStart = std::hypot(currentRow.start.x - firstRowStart.x, currentRow.start.y - firstRowStart.y);
    double distToEnd = std::hypot(currentRow.end.x - firstRowStart.x, currentRow.end.y - firstRowStart.y);
    
    Point closestPoint;
    Point farthestPoint;
    if (distToStart <= distToEnd) {
        closestPoint = currentRow.start;
        farthestPoint = currentRow.end;
    } else {
        closestPoint = currentRow.end;
        farthestPoint = currentRow.start;
    }
    
    // Calculer la direction depuis le point le plus éloigné vers le point le plus proche
    double dx = closestPoint.x - farthestPoint.x;
    double dy = closestPoint.y - farthestPoint.y;
    double length = std::sqrt(dx*dx + dy*dy);
    
    if (length < 0.001) {
        // Si les points sont confondus, on utilise une direction par défaut
        dx = 1.0; dy = 0.0;
        length = 1.0;
    }
    
    // Normaliser le vecteur direction
    dx /= length;
    dy /= length;
    
    // Aller au point le plus proche et CONTINUER de 4m dans la même direction (vers l'extérieur)
    Point extended_point = {
        closestPoint.x + dx * extra_distance,
        closestPoint.y + dy * extra_distance
    };
    
    return extended_point;
}

// Fonction pour créer une PoseStamped à partir d'un Point
geometry_msgs::msg::PoseStamped createPoseFromPoint(const Point& point, const std::string& frame_id = "map") {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.stamp = rclcpp::Clock().now();
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = 0.0;
    
    // Orientation par défaut (face à la première rangée)
    tf2::Quaternion q;
    q.setRPY(0, 0, 0); // Vous pouvez ajuster l'orientation si nécessaire
    pose.pose.orientation = tf2::toMsg(q);
    
    return pose;
}

std::vector<Row> parseFieldRows(const std::string& fieldFile) {
    std::vector<Row> rows;

    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(fieldFile.c_str()) != tinyxml2::XML_SUCCESS) {
        std::cerr << "Error parsing field file: " << doc.ErrorStr() << "\n";
        return rows;
    }

    tinyxml2::XMLElement* root = doc.RootElement();
    if (!root) {
        std::cerr << "No root element found.\n";
        return rows;
    }

    for (tinyxml2::XMLElement* rowElem = root->FirstChildElement("Row");
         rowElem != nullptr;
         rowElem = rowElem->NextSiblingElement("Row")) {

        int rowId = rowElem->IntAttribute("id");

        tinyxml2::XMLElement* coordsElem = rowElem->FirstChildElement("geometry")
                                                   ->FirstChildElement("gml:LineString")
                                                   ->FirstChildElement("gml:coordinates");

        if (coordsElem && coordsElem->GetText()) {
            std::string coordsText = coordsElem->GetText();
            std::istringstream iss(coordsText);
            std::vector<std::string> points;
            std::string token;
            while (iss >> token) points.push_back(token);

            if (points.size() >= 2) {
                Point start = parsePoint(points.front());
                Point end   = parsePoint(points.back());
                rows.push_back({rowId, start, end});
                std::cout << "Parsed row " << rowId
                          << ": START(" << start.x << "," << start.y << ") -> END("
                          << end.x << "," << end.y << ")\n";
            }
        }
    }

    std::cout << "Successfully parsed " << rows.size() << " rows from field file\n";
    return rows;
}

class GoToLoadingStation : public BT::SyncActionNode {
public:
    GoToLoadingStation(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config),
      node_(rclcpp::Node::make_shared("go_to_loading_station")),
      tf_buffer_(node_->get_clock()),
      tf_listener_(tf_buffer_)
    {
        path_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("GoToLoadingStationPath", 5);
        rows = parseFieldRows(xmlFile);
        
        // Stocker le point de départ de la première rangée
        if (!rows.empty()) {
            firstRowStart_ = rows[0].start;
            RCLCPP_INFO(node_->get_logger(), "First row start point: (%.2f, %.2f)", 
                       firstRowStart_.x, firstRowStart_.y);
        }
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<nav_msgs::msg::Path>("Navigation_path"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose"),
            BT::OutputPort<int>("closest_row_id"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("Loading_station_pose")  
        };
    }

    BT::NodeStatus tick() override {
        // Get robot pose
        geometry_msgs::msg::TransformStamped transformStamped;
        try {
            transformStamped = tf_buffer_.lookupTransform("map", "base_link", 
                                                         tf2::TimePointZero, 
                                                         tf2::durationFromSec(0.1));
        } catch (tf2::TransformException& ex) {
            RCLCPP_ERROR(node_->get_logger(), "TF error: %s", ex.what());
            return BT::NodeStatus::FAILURE;
        }

        geometry_msgs::msg::PoseStamped robot_pose;
        robot_pose.header.frame_id = "map";
        robot_pose.header.stamp = node_->now();
        robot_pose.pose.position.x = transformStamped.transform.translation.x;
        robot_pose.pose.position.y = transformStamped.transform.translation.y;
        robot_pose.pose.position.z = transformStamped.transform.translation.z;
        robot_pose.pose.orientation = transformStamped.transform.rotation;

        // Check in which row is the robot to know where to go
        Point robotPos = {robot_pose.pose.position.x, robot_pose.pose.position.y};
        int currentRowId = detectCurrentRow(robotPos, rows, 1.0);
        
        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = node_->now();
        
        // Calculer la Loading_station_pose (toujours la première rangée + 4m)
        geometry_msgs::msg::PoseStamped Loading_station_pose;
        if (!rows.empty()) {
            Point loadingPoint = getExitPointTowardsFirstRowStart(rows[0], firstRowStart_, 4.0);
            Loading_station_pose = createPoseFromPoint(loadingPoint, "map");
            
            // Définir l'orientation pour la loading pose
            double dx = loadingPoint.x - rows[0].start.x;
            double dy = loadingPoint.y - rows[0].start.y;
            double length = std::sqrt(dx*dx + dy*dy);
            if (length > 0.001) {
                dx /= length; dy /= length;
            }
            double yaw = std::atan2(dy, dx);
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            Loading_station_pose.pose.orientation = tf2::toMsg(q);
        }
        
        if (currentRowId != -1) {
            RCLCPP_INFO(node_->get_logger(), "Robot est dans la Row #%d", currentRowId);
            
            // Trouver la rangée correspondante
            const Row* targetRow = nullptr;
            for (const auto& row : rows) {
                if (row.id == currentRowId) {
                    targetRow = &row;
                    break;
                }
            }
            
            if (targetRow) {
                // NOUVELLE LOGIQUE : Aller à 4m APRÈS le point de sortie
                Point exitPoint = getExitPointTowardsFirstRowStart(*targetRow, firstRowStart_, 4.0);
                
                RCLCPP_INFO(node_->get_logger(), "Row %d détectée: Start(%.2f,%.2f) End(%.2f,%.2f)", 
                           targetRow->id, targetRow->start.x, targetRow->start.y, 
                           targetRow->end.x, targetRow->end.y);
                RCLCPP_INFO(node_->get_logger(), "Going to exit point + 4m: (%.2f, %.2f)", 
                           exitPoint.x, exitPoint.y);
                
                goal.pose.position.x = exitPoint.x;
                goal.pose.position.y = exitPoint.y;
                
                // Calculer l'orientation pour regarder vers l'extérieur
                double dx = exitPoint.x - targetRow->start.x;
                double dy = exitPoint.y - targetRow->start.y;
                double length = std::sqrt(dx*dx + dy*dy);
                
                if (length < 0.001) {
                    dx = 1.0; dy = 0.0;
                } else {
                    dx /= length; dy /= length;
                }
                
                double yaw = std::atan2(dy, dx);
                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);
                goal.pose.orientation = tf2::toMsg(q);
                
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Row %d not found in parsed rows!", currentRowId);
                applyForwardOffset(robot_pose, goal, 4.0);
            }
        } else {
            RCLCPP_WARN(node_->get_logger(), "Robot n'est pas dans une Row détectée");
            applyForwardOffset(robot_pose, goal, 4.0);
        }
        
        goal.pose.position.z = 0.0;
        
        // Set output ports
        setOutput("goal_pose", goal);
        setOutput("Loading_station_pose", Loading_station_pose);
        setOutput("closest_row_id", currentRowId);
        
        path_pub_->publish(goal);
        RCLCPP_INFO(node_->get_logger(), "Goal envoyé: (%.2f, %.2f) - 4m après le point de sortie", 
                   goal.pose.position.x, goal.pose.position.y);
        RCLCPP_INFO(node_->get_logger(), "Loading pose définie: (%.2f, %.2f)", 
                   Loading_station_pose.pose.position.x, Loading_station_pose.pose.position.y);
        
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr path_pub_;
    std::string xmlFile = "/home/hedi/eterry_simulation/src/eterry_sim_stack/simulation_navigation/maps/output.xml";
    std::vector<Row> rows;
    Point firstRowStart_ = {0.0, 0.0};

    void applyForwardOffset(const geometry_msgs::msg::PoseStamped& robot_pose, 
                           geometry_msgs::msg::PoseStamped& goal, double distance) {
        tf2::Quaternion q(
            robot_pose.pose.orientation.x,
            robot_pose.pose.orientation.y,
            robot_pose.pose.orientation.z,
            robot_pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        goal.pose.position.x = robot_pose.pose.position.x + std::cos(yaw) * distance;
        goal.pose.position.y = robot_pose.pose.position.y + std::sin(yaw) * distance;
        goal.pose.orientation = robot_pose.pose.orientation;
    }

    int detectCurrentRow(const Point& position, const std::vector<Row>& rows, double tolerance) {
        // Essayer d'abord avec la tolérance normale
        for (const auto& row : rows) {
            if (Geometry::isPointInBox(position, row.start, row.end, tolerance)) {
                RCLCPP_INFO(node_->get_logger(), "Robot dans Row %d", row.id);
                return row.id;
            }
        }
        
        // Si pas trouvé, essayer avec une tolérance plus large
        for (const auto& row : rows) {
            if (Geometry::isPointInBox(position, row.start, row.end, tolerance * 2.0)) {
                RCLCPP_WARN(node_->get_logger(), "Robot dans Row %d (tolérance étendue)", row.id);
                return row.id;
            }
        }
        
        // Si toujours pas trouvé, trouver la rangée la plus proche
        double minDistance = std::numeric_limits<double>::max();
        int closestRowId = -1;
        
        for (const auto& row : rows) {
            double distance = distanceToLineSegment(position, row.start, row.end);
            if (distance < minDistance) {
                minDistance = distance;
                closestRowId = row.id;
            }
        }
        
        if (closestRowId != -1 && minDistance < 5.0) {
            RCLCPP_WARN(node_->get_logger(), "Row la plus proche: %d (distance: %.2fm)", closestRowId, minDistance);
            return closestRowId;
        }
        
        RCLCPP_ERROR(node_->get_logger(), "Aucune Row détectée à proximité");
        return -1;
    }

    double distanceToLineSegment(const Point& p, const Point& a, const Point& b) {
        double abx = b.x - a.x;
        double aby = b.y - a.y;
        double apx = p.x - a.x;
        double apy = p.y - a.y;
        
        double dot = apx * abx + apy * aby;
        double ab_len_sq = abx * abx + aby * aby;
        
        if (ab_len_sq < 1e-10) {
            return std::hypot(p.x - a.x, p.y - a.y);
        }
        
        double t = std::max(0.0, std::min(1.0, dot / ab_len_sq));
        Point closest = {a.x + t * abx, a.y + t * aby};
        
        return std::hypot(p.x - closest.x, p.y - closest.y);
    }
};

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<GoToLoadingStation>("GoToLoadingStation");
}