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

// Nouvelle fonction pour déterminer le côté de sortie basé sur le premier row
Point getExitPointConsistentSide(const Row& currentRow, const Row& firstRow, double extra_distance = 2.0) {
    // Déterminer si le premier row est horizontal ou vertical
    bool isFirstRowHorizontal = std::abs(firstRow.end.y - firstRow.start.y) < std::abs(firstRow.end.x - firstRow.start.x);
    
    Point exitPoint;
    
    if (isFirstRowHorizontal) {
        // Map horizontale : sortir du côté du start du premier row (côté gauche/right)
        if (firstRow.start.x <= firstRow.end.x) {
            // Premier row va de gauche à droite → sortir par le côté gauche (start du premier row)
            exitPoint.x = currentRow.start.x;
            exitPoint.y = currentRow.start.y;
        } else {
            // Premier row va de droite à gauche → sortir par le côté droit (start du premier row)
            exitPoint.x = currentRow.end.x;
            exitPoint.y = currentRow.end.y;
        }
    } else {
        // Map verticale : sortir du côté du start du premier row (côté haut/bas)
        if (firstRow.start.y <= firstRow.end.y) {
            // Premier row va de bas en haut → sortir par le bas (start du premier row)
            exitPoint.x = currentRow.start.x;
            exitPoint.y = currentRow.start.y;
        } else {
            // Premier row va de haut en bas → sortir par le haut (start du premier row)
            exitPoint.x = currentRow.end.x;
            exitPoint.y = currentRow.end.y;
        }
    }
    
    // Calculer la direction pour l'offset
    double dx, dy;
    if (isFirstRowHorizontal) {
        // Pour les maps horizontales, offset perpendiculaire aux rows
        double row_dy = currentRow.end.y - currentRow.start.y;
        double row_dx = currentRow.end.x - currentRow.start.x;
        double length = std::sqrt(row_dx*row_dx + row_dy*row_dy);
        if (length > 0.001) {
            // Vecteur perpendiculaire (rotation de 90 degrés)
            dx = -row_dy / length;
            dy = row_dx / length;
            
            // Ajuster la direction selon le côté de sortie du premier row
            if (firstRow.start.x <= firstRow.end.x) {
                // Sortie côté gauche → offset vers la gauche
                exitPoint.x -= dx * extra_distance;
                exitPoint.y -= dy * extra_distance;
            } else {
                // Sortie côté droit → offset vers la droite  
                exitPoint.x += dx * extra_distance;
                exitPoint.y += dy * extra_distance;
            }
        }
    } else {
        // Pour les maps verticales, offset perpendiculaire aux rows
        double row_dy = currentRow.end.y - currentRow.start.y;
        double row_dx = currentRow.end.x - currentRow.start.x;
        double length = std::sqrt(row_dx*row_dx + row_dy*row_dy);
        if (length > 0.001) {
            // Vecteur perpendiculaire (rotation de 90 degrés)
            dx = -row_dy / length;
            dy = row_dx / length;
            
            // Ajuster la direction selon le côté de sortie du premier row
            if (firstRow.start.y <= firstRow.end.y) {
                // Sortie côté bas → offset vers le bas
                exitPoint.x -= dx * extra_distance;
                exitPoint.y -= dy * extra_distance;
            } else {
                // Sortie côté haut → offset vers le haut
                exitPoint.x += dx * extra_distance;
                exitPoint.y += dy * extra_distance;
            }
        }
    }
    
    return exitPoint;
}

// Fonction pour calculer l'orientation de sortie
double calculateExitOrientation(const Row& currentRow, const Row& firstRow) {
    bool isFirstRowHorizontal = std::abs(firstRow.end.y - firstRow.start.y) < std::abs(firstRow.end.x - firstRow.start.x);
    
    double dx, dy;
    
    if (isFirstRowHorizontal) {
        if (firstRow.start.x <= firstRow.end.x) {
            // Sortie côté gauche → regarder vers l'ouest (ou direction opposée à la rangée)
            dx = -1.0;
            dy = 0.0;
        } else {
            // Sortie côté droit → regarder vers l'est (ou direction de la rangée)
            dx = 1.0;
            dy = 0.0;
        }
    } else {
        if (firstRow.start.y <= firstRow.end.y) {
            // Sortie côté bas → regarder vers le sud
            dx = 0.0;
            dy = -1.0;
        } else {
            // Sortie côté haut → regarder vers le nord
            dx = 0.0;
            dy = 1.0;
        }
    }
    
    return std::atan2(dy, dx);
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
        if (!rows.empty()) {
            firstRow = rows[0];
            RCLCPP_INFO(node_->get_logger(), "First row %d: Start(%.2f,%.2f) End(%.2f,%.2f)", 
                       firstRow.id, firstRow.start.x, firstRow.start.y, firstRow.end.x, firstRow.end.y);
        }
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<nav_msgs::msg::Path>("Navigation_path"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal_pose"),
            BT::OutputPort<int>("closest_row_id")  
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
        
        if (currentRowId != -1 && !rows.empty()) {
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
                // Utiliser la nouvelle logique de sortie cohérente
                Point exitPoint = getExitPointConsistentSide(*targetRow, firstRow, 2.0);
                double exitYaw = calculateExitOrientation(*targetRow, firstRow);
                
                RCLCPP_INFO(node_->get_logger(), "Row %d détectée: Start(%.2f,%.2f) End(%.2f,%.2f)", 
                        targetRow->id, targetRow->start.x, targetRow->start.y, 
                        targetRow->end.x, targetRow->end.y);
                RCLCPP_INFO(node_->get_logger(), "Sortie côté cohérent: (%.2f, %.2f)", 
                        exitPoint.x, exitPoint.y);
                
                goal.pose.position.x = exitPoint.x;
                goal.pose.position.y = exitPoint.y;
                
                tf2::Quaternion q;
                q.setRPY(0, 0, exitYaw);
                goal.pose.orientation = tf2::toMsg(q);
                
                RCLCPP_INFO(node_->get_logger(), "Goal position: (%.2f, %.2f), orientation: %.2f rad", 
                        goal.pose.position.x, goal.pose.position.y, exitYaw);
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Row %d not found in parsed rows!", currentRowId);
                applyForwardOffset(robot_pose, goal, 2.0);
            }
        } else {
            RCLCPP_WARN(node_->get_logger(), "Robot n'est pas dans une Row détectée");
            applyForwardOffset(robot_pose, goal, 2.0);
        }
        
        goal.pose.position.z = 0.0;
        
        // Set output port
        setOutput("goal_pose", goal);
        path_pub_->publish(goal);
        RCLCPP_INFO(node_->get_logger(), "Goal envoyé: (%.2f, %.2f)", 
                   goal.pose.position.x, goal.pose.position.y);
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr path_pub_;
    std::string xmlFile = "/home/hedi/eterry_simulation/src/eterry_sim_stack/simulation_navigation/maps/output.xml";
    std::vector<Row> rows;
    Row firstRow;

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
        setOutput("closest_row_id", closestRowId);
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