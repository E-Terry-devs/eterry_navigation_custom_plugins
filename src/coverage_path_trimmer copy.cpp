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
#include <vector>
#include <cmath>
#include <algorithm>

// Structures pour représenter les rows
struct Point {
    double x, y;
    
    std::string toString() const {
        return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
    }
};

struct Row {
    int id;
    Point start;
    Point end;
    
    Point getBottomPoint() const {
        return (start.y > end.y) ? start : end;
    }
    
    std::string toString() const {
        return "Row " + std::to_string(id) + ": start" + start.toString() + " end" + end.toString();
    }
};

class CoveragePathTrimmer : public BT::SyncActionNode
{
public:
  CoveragePathTrimmer(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("coverage_path_trimmer_node");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Charger les rows au démarrage
    rows_ = parseFieldRows("/home/hedi/eterry_simulation/src/eterry_sim_stack/simulation_navigation/maps/output.xml");
    
    RCLCPP_INFO(node_->get_logger(), "CoveragePathTrimmer initialized with %zu rows", rows_.size());
    for (const auto& row : rows_) {
        RCLCPP_DEBUG(node_->get_logger(), "  %s", row.toString().c_str());
    }
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("nav_path"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("interrupted_pose"),
      BT::OutputPort<nav_msgs::msg::Path>("trimmed_path")
    };
  }
  
  BT::NodeStatus tick() override
  {
    RCLCPP_INFO(node_->get_logger(), "=== CoveragePathTrimmer START ===");
    
    // Get input path
    nav_msgs::msg::Path input_path;
    if (!getInput("nav_path", input_path)) {
      RCLCPP_ERROR(node_->get_logger(), "Missing required input [nav_path]");
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "Input path has %zu poses", input_path.poses.size());
    if (input_path.poses.empty()) {
      RCLCPP_WARN(node_->get_logger(), "Input path is empty");
      return BT::NodeStatus::FAILURE;
    }

    // Afficher les premiers points du path pour debug
    for (size_t i = 0; i < std::min(input_path.poses.size(), size_t(3)); ++i) {
        RCLCPP_DEBUG(node_->get_logger(), "Path point %zu: (%.2f, %.2f)", 
                    i, input_path.poses[i].pose.position.x, input_path.poses[i].pose.position.y);
    }

    // Get current robot pose from TF
    geometry_msgs::msg::PoseStamped current_robot_pose;
    if (!getCurrentRobotPose(current_robot_pose)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get current robot pose from TF");
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "Current robot pose: (%.2f, %.2f)", 
                current_robot_pose.pose.position.x, current_robot_pose.pose.position.y);

    nav_msgs::msg::Path trimmed_path;
    trimmed_path.header = input_path.header;

    // Check if CURRENT robot pose is within the coverage path
    bool is_within_path = isRobotInCoveragePath(current_robot_pose, input_path);
    
    RCLCPP_INFO(node_->get_logger(), "Robot is within coverage path: %s", is_within_path ? "YES" : "NO");
    
    if (is_within_path) {
      // Cas 1: Robot actuel est dans le coverage path - trimming standard
      RCLCPP_INFO(node_->get_logger(), "=== USING STANDARD TRIMMING ===");
      
      size_t next_idx = findNextWaypoint(current_robot_pose, input_path);
      RCLCPP_INFO(node_->get_logger(), "Next waypoint index: %zu/%zu", next_idx, input_path.poses.size());
      
      if (next_idx >= input_path.poses.size()) {
        RCLCPP_INFO(node_->get_logger(), "Reached end of path");
        return BT::NodeStatus::FAILURE;
      }

      trimmed_path.poses.assign(input_path.poses.begin() + next_idx, input_path.poses.end());
      RCLCPP_INFO(node_->get_logger(), "Standard trimming: starting from waypoint %zu", next_idx);
    }
    else {
      // Cas 2: Robot actuel est hors du coverage path
      RCLCPP_INFO(node_->get_logger(), "=== USING OUTSIDE PATH STRATEGY ===");
      
      // Trouver l'index cible basé sur interrupted_pose ou position actuelle
      geometry_msgs::msg::PoseStamped target_pose;
      bool has_interrupted_pose = getInput("interrupted_pose", has_interrupted_pose).has_value();
      
      if (!has_interrupted_pose) {
        target_pose = current_robot_pose;
        RCLCPP_INFO(node_->get_logger(), "No interrupted pose available, using current pose as target");
      } else {
        RCLCPP_INFO(node_->get_logger(), "Using interrupted pose to find target row");
        RCLCPP_INFO(node_->get_logger(), "Interrupted pose: (%.2f, %.2f)", 
                   target_pose.pose.position.x, target_pose.pose.position.y);
      }
      
      size_t target_idx = findInterruptedIndex(target_pose, input_path);
      RCLCPP_INFO(node_->get_logger(), "Target index in path: %zu/%zu", target_idx, input_path.poses.size());
      
      if (target_idx >= input_path.poses.size()) {
        RCLCPP_WARN(node_->get_logger(), "Could not find target position in path, using full path");
        trimmed_path = input_path;
      }
      else {
        // Afficher le point cible dans le path
        RCLCPP_INFO(node_->get_logger(), "Target point in path: (%.2f, %.2f)", 
                   input_path.poses[target_idx].pose.position.x, 
                   input_path.poses[target_idx].pose.position.y);
        
        // Trouver le début de la row appropriée
        size_t row_start_idx = findRowStart(target_idx, input_path);
        RCLCPP_INFO(node_->get_logger(), "Row start index: %zu/%zu", row_start_idx, input_path.poses.size());
        
        // Afficher le point de départ de la row
        RCLCPP_INFO(node_->get_logger(), "Row start point: (%.2f, %.2f)", 
                   input_path.poses[row_start_idx].pose.position.x, 
                   input_path.poses[row_start_idx].pose.position.y);
        
        // Vérifier la distance pour la sécurité
        double distance_to_row = pointDistance(current_robot_pose.pose.position, 
                                             input_path.poses[row_start_idx].pose.position);
        RCLCPP_INFO(node_->get_logger(), "Distance to row: %.2f meters", distance_to_row);
        
        if (distance_to_row > MAX_SAFE_DISTANCE) {
          RCLCPP_WARN(node_->get_logger(), "Distance to row (%.2fm) exceeds safe limit (%.2fm), using full path", 
                     distance_to_row, MAX_SAFE_DISTANCE);
          trimmed_path = input_path;
        }
        else {
          // Générer un chemin sécurisé vers la row
          RCLCPP_INFO(node_->get_logger(), "Generating safe path to row...");
          trimmed_path = generateSafePathToRow(current_robot_pose, 
                                             input_path.poses[row_start_idx], 
                                             input_path, 
                                             row_start_idx);
          
          RCLCPP_INFO(node_->get_logger(), "Generated safe path with %zu points", trimmed_path.poses.size());
          
          // Afficher les points générés pour debug
          for (size_t i = 0; i < std::min(trimmed_path.poses.size(), size_t(5)); ++i) {
              RCLCPP_DEBUG(node_->get_logger(), "Generated point %zu: (%.2f, %.2f)", 
                          i, trimmed_path.poses[i].pose.position.x, trimmed_path.poses[i].pose.position.y);
          }
        }
      }
    }

    // Validation finale du chemin
    if (trimmed_path.poses.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Generated path is empty, falling back to original path");
      trimmed_path = input_path;
    }

    setOutput("trimmed_path", trimmed_path);
    RCLCPP_INFO(node_->get_logger(), "=== CoveragePathTrimmer END ===");
    RCLCPP_INFO(node_->get_logger(), "Final trimmed path: %zu waypoints", trimmed_path.poses.size());

    return BT::NodeStatus::SUCCESS;
  }

private:
  std::vector<Row> rows_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Constants for safety
  const double MAX_SAFE_DISTANCE = 20.0; // meters
  const double WAYPOINT_SPACING = 0.5; // meters between intermediate points
  const double PROXIMITY_THRESHOLD = 0.1; // meters to consider robot "in path"

  // Get current robot pose from TF
  bool getCurrentRobotPose(geometry_msgs::msg::PoseStamped& robot_pose) const {
    try {
      geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
        "map", "base_link", tf2::TimePointZero, std::chrono::seconds(1));
      
      robot_pose.header.stamp = node_->now();
      robot_pose.header.frame_id = "map";
      robot_pose.pose.position.x = transform.transform.translation.x;
      robot_pose.pose.position.y = transform.transform.translation.y;
      robot_pose.pose.position.z = transform.transform.translation.z;
      robot_pose.pose.orientation = transform.transform.rotation;
      
      return true;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(node_->get_logger(), "TF exception: %s", ex.what());
      return false;
    }
  }

  // Generate safe path with intermediate points
  nav_msgs::msg::Path generateSafePathToRow(const geometry_msgs::msg::PoseStamped& start_pose,
                                          const geometry_msgs::msg::PoseStamped& row_start_pose,
                                          const nav_msgs::msg::Path& original_path,
                                          size_t row_start_idx) const {
    RCLCPP_DEBUG(node_->get_logger(), "generateSafePathToRow called");
    RCLCPP_DEBUG(node_->get_logger(), "Start pose: (%.2f, %.2f)", 
                start_pose.pose.position.x, start_pose.pose.position.y);
    RCLCPP_DEBUG(node_->get_logger(), "Row start pose: (%.2f, %.2f)", 
                row_start_pose.pose.position.x, row_start_pose.pose.position.y);
    RCLCPP_DEBUG(node_->get_logger(), "Row start index: %zu", row_start_idx);
    
    nav_msgs::msg::Path safe_path;
    safe_path.header = original_path.header;
    
    // Add current position as first point
    safe_path.poses.push_back(start_pose);
    RCLCPP_DEBUG(node_->get_logger(), "Added start pose to path");
    
    // Calculate total distance
    double total_distance = pointDistance(start_pose.pose.position, row_start_pose.pose.position);
    RCLCPP_DEBUG(node_->get_logger(), "Total distance to row: %.2f meters", total_distance);
    
    // Generate intermediate points if distance is significant
    if (total_distance > WAYPOINT_SPACING) {
      int num_intermediate_points = std::max(1, static_cast<int>(total_distance / WAYPOINT_SPACING));
      RCLCPP_DEBUG(node_->get_logger(), "Generating %d intermediate points", num_intermediate_points);
      
      for (int i = 1; i <= num_intermediate_points; ++i) {
        double ratio = static_cast<double>(i) / (num_intermediate_points + 1);
        
        geometry_msgs::msg::PoseStamped intermediate_pose = start_pose;
        intermediate_pose.pose.position.x = start_pose.pose.position.x + 
                                           (row_start_pose.pose.position.x - start_pose.pose.position.x) * ratio;
        intermediate_pose.pose.position.y = start_pose.pose.position.y + 
                                           (row_start_pose.pose.position.y - start_pose.pose.position.y) * ratio;
        
        // Maintain the same orientation as the row start point for better navigation
        if (i == num_intermediate_points) {
          intermediate_pose.pose.orientation = row_start_pose.pose.orientation;
        }
        
        safe_path.poses.push_back(intermediate_pose);
        RCLCPP_DEBUG(node_->get_logger(), "Added intermediate point %d: (%.2f, %.2f)", 
                    i, intermediate_pose.pose.position.x, intermediate_pose.pose.position.y);
      }
    } else {
      RCLCPP_DEBUG(node_->get_logger(), "Distance is small, no intermediate points needed");
    }
    
    // Add the row start point
    safe_path.poses.push_back(row_start_pose);
    RCLCPP_DEBUG(node_->get_logger(), "Added row start pose to path");
    
    // Add the remaining path from the row start
    if (row_start_idx + 1 < original_path.poses.size()) {
      size_t points_added = original_path.poses.size() - (row_start_idx + 1);
      safe_path.poses.insert(safe_path.poses.end(),
                           original_path.poses.begin() + row_start_idx + 1,
                           original_path.poses.end());
      RCLCPP_DEBUG(node_->get_logger(), "Added %zu points from original path", points_added);
    } else {
      RCLCPP_DEBUG(node_->get_logger(), "No additional points to add from original path");
    }
    
    RCLCPP_INFO(node_->get_logger(), "Safe path generation complete: %zu total points", safe_path.poses.size());
    
    return safe_path;
  }

  // Parser pour extraire les coordonnées d'un point
  Point parsePoint(const std::string& pointStr) const {
      Point p;
      std::string cleaned = pointStr;
      std::replace(cleaned.begin(), cleaned.end(), ',', ' ');
      std::istringstream iss(cleaned);
      iss >> p.x >> p.y;
      return p;
  }

  // Parser pour extraire les rows du fichier XML
  std::vector<Row> parseFieldRows(const std::string& fieldFile) const {
      std::vector<Row> rows;

      tinyxml2::XMLDocument doc;
      if (doc.LoadFile(fieldFile.c_str()) != tinyxml2::XML_SUCCESS) {
          RCLCPP_ERROR(node_->get_logger(), "Error loading field file: %s - %s", 
                      fieldFile.c_str(), doc.ErrorStr());
          return rows;
      }

      tinyxml2::XMLElement* root = doc.RootElement();
      if (!root) {
          RCLCPP_ERROR(node_->get_logger(), "No root element found in field file");
          return rows;
      }

      for (tinyxml2::XMLElement* rowElem = root->FirstChildElement("Row");
           rowElem != nullptr;
           rowElem = rowElem->NextSiblingElement("Row")) {

          int rowId = 0;
          if (rowElem->QueryIntAttribute("id", &rowId) != tinyxml2::XML_SUCCESS) {
              continue;
          }

          tinyxml2::XMLElement* geometryElem = rowElem->FirstChildElement("geometry");
          if (!geometryElem) continue;
          
          tinyxml2::XMLElement* lineStringElem = geometryElem->FirstChildElement("gml:LineString");
          if (!lineStringElem) continue;
          
          tinyxml2::XMLElement* coordsElem = lineStringElem->FirstChildElement("gml:coordinates");
          if (!coordsElem || !coordsElem->GetText()) continue;

          std::string coordsText = coordsElem->GetText();
          std::istringstream iss(coordsText);
          std::vector<std::string> points;
          std::string token;
          
          while (iss >> token) {
              if (!token.empty()) {
                  points.push_back(token);
              }
          }

          if (points.size() >= 2) {
              Point start = parsePoint(points.front());
              Point end = parsePoint(points.back());
              rows.push_back({rowId, start, end});
          }
      }
      
      RCLCPP_INFO(node_->get_logger(), "Loaded %zu rows from field file", rows.size());
      return rows;
  }

  // Convertir Point personnalisé en geometry_msgs::Point
  geometry_msgs::msg::Point toPointMsg(const Point& p) const {
      geometry_msgs::msg::Point point_msg;
      point_msg.x = p.x;
      point_msg.y = p.y;
      point_msg.z = 0.0;
      return point_msg;
  }

  // Trouver la row la plus proche d'un point donné
  int findClosestRow(const geometry_msgs::msg::PoseStamped& pose) const {
      if (rows_.empty()) {
          RCLCPP_DEBUG(node_->get_logger(), "No rows available for closest row detection");
          return -1;
      }
      
      int closest_row_id = -1;
      double min_distance = std::numeric_limits<double>::max();
      
      RCLCPP_DEBUG(node_->get_logger(), "Finding closest row to point (%.2f, %.2f)", 
                  pose.pose.position.x, pose.pose.position.y);
      
      for (const auto& row : rows_) {
          double dist_to_start = pointDistance(pose.pose.position, toPointMsg(row.start));
          double dist_to_end = pointDistance(pose.pose.position, toPointMsg(row.end));
          double min_dist_to_row = std::min(dist_to_start, dist_to_end);
          
          RCLCPP_DEBUG(node_->get_logger(), "Row %d: dist_to_start=%.2f, dist_to_end=%.2f, min=%.2f", 
                      row.id, dist_to_start, dist_to_end, min_dist_to_row);
          
          if (min_dist_to_row < min_distance) {
              min_distance = min_dist_to_row;
              closest_row_id = row.id;
          }
      }
      
      RCLCPP_DEBUG(node_->get_logger(), "Closest row: %d (distance: %.2f)", closest_row_id, min_distance);
      return closest_row_id;
  }

  // Trouver le point de départ d'une row dans le path
  size_t findRowStartInPath(int row_id, const nav_msgs::msg::Path& path) const {
      if (rows_.empty() || path.poses.empty()) {
          RCLCPP_WARN(node_->get_logger(), "No rows or path is empty, returning index 0");
          return 0;
      }
      
      Row target_row;
      bool found = false;
      for (const auto& row : rows_) {
          if (row.id == row_id) {
              target_row = row;
              found = true;
              break;
          }
      }
      
      if (!found) {
          RCLCPP_WARN(node_->get_logger(), "Row %d not found in loaded rows, returning index 0", row_id);
          return 0;
      }
      
      Point row_start_point = target_row.getBottomPoint();
      RCLCPP_DEBUG(node_->get_logger(), "Looking for row %d start point: %s", 
                  row_id, row_start_point.toString().c_str());
      
      size_t closest_idx = 0;
      double min_distance = std::numeric_limits<double>::max();
      
      for (size_t i = 0; i < path.poses.size(); ++i) {
          double distance = pointDistance(path.poses[i].pose.position, toPointMsg(row_start_point));
          
          if (distance < min_distance) {
              min_distance = distance;
              closest_idx = i;
          }
          
          if (distance < 0.5) {
              RCLCPP_DEBUG(node_->get_logger(), "Found close match at index %zu (distance: %.3f)", i, distance);
              break;
          }
      }
      
      RCLCPP_INFO(node_->get_logger(), "Row %d start found at index %zu (distance: %.3f)", 
                 row_id, closest_idx, min_distance);
      
      return closest_idx;
  }

  // Trouver le début de la row appropriée
  size_t findRowStart(size_t target_idx, const nav_msgs::msg::Path& path) const {
      RCLCPP_DEBUG(node_->get_logger(), "findRowStart called with target_idx: %zu", target_idx);
      
      if (rows_.empty()) {
          RCLCPP_WARN(node_->get_logger(), "No rows available, using angle-based detection");
          return findRowStartByAngle(target_idx, path);
      }
      
      if (target_idx >= path.poses.size()) {
          RCLCPP_WARN(node_->get_logger(), "Target index %zu out of bounds, using last index", target_idx);
          target_idx = path.poses.size() - 1;
      }
      
      geometry_msgs::msg::PoseStamped target_pose = path.poses[target_idx];
      int closest_row_id = findClosestRow(target_pose);
      
      if (closest_row_id == -1) {
          RCLCPP_WARN(node_->get_logger(), "No closest row found, using angle-based detection");
          return findRowStartByAngle(target_idx, path);
      }
      
      RCLCPP_INFO(node_->get_logger(), "Target detected in row %d", closest_row_id);
      return findRowStartInPath(closest_row_id, path);
  }

  // Méthode fallback par détection d'angle
  size_t findRowStartByAngle(size_t target_idx, const nav_msgs::msg::Path& path) const {
      RCLCPP_DEBUG(node_->get_logger(), "findRowStartByAngle called with target_idx: %zu", target_idx);
      
      double angle_threshold = 0.5;
      size_t start_index = std::min(target_idx, path.poses.size() - 1);
      
      RCLCPP_DEBUG(node_->get_logger(), "Starting angle detection from index: %zu", start_index);
      
      // Start from target index and go backwards to find row start
      for (size_t i = start_index; i >= 2; --i) {
          double dx1 = path.poses[i].pose.position.x - path.poses[i-1].pose.position.x;
          double dy1 = path.poses[i].pose.position.y - path.poses[i-1].pose.position.y;
          double dx2 = path.poses[i-1].pose.position.x - path.poses[i-2].pose.position.x;
          double dy2 = path.poses[i-1].pose.position.y - path.poses[i-2].pose.position.y;
          
          double angle1 = atan2(dy1, dx1);
          double angle2 = atan2(dy2, dx2);
          double angle_diff = fabs(angle1 - angle2);
          
          if (angle_diff > M_PI) {
              angle_diff = 2 * M_PI - angle_diff;
          }
          
          if (angle_diff > angle_threshold) {
              RCLCPP_INFO(node_->get_logger(), "Angle change detected at index %zu: %.3f rad", i, angle_diff);
              return i;
          }
          
          double dist = pointDistance(path.poses[i].pose.position, path.poses[i-1].pose.position);
          if (dist > 3.0) {
              RCLCPP_INFO(node_->get_logger(), "Distance gap detected at index %zu: %.3f m", i, dist);
              return i;
          }
      }
      
      size_t result = (target_idx > 0) ? target_idx : 0;
      RCLCPP_INFO(node_->get_logger(), "No significant angle/distance changes found, using index: %zu", result);
      return result;
  }

  bool isRobotInCoveragePath(const geometry_msgs::msg::PoseStamped& robot_pose, 
                           const nav_msgs::msg::Path& path) const {
      double min_distance = std::numeric_limits<double>::max();
      
      for (const auto& pose : path.poses) {
          double distance = pointDistance(robot_pose.pose.position, pose.pose.position);
          min_distance = std::min(min_distance, distance);
          
          if (distance < PROXIMITY_THRESHOLD) {
              RCLCPP_DEBUG(node_->get_logger(), "Robot within path: distance %.2f < threshold %.2f", 
                          distance, PROXIMITY_THRESHOLD);
              return true;
          }
      }
      
      RCLCPP_DEBUG(node_->get_logger(), "Robot outside path: min distance %.2f >= threshold %.2f", 
                  min_distance, PROXIMITY_THRESHOLD);
      return false;
  }

  size_t findInterruptedIndex(const geometry_msgs::msg::PoseStamped& target_pose,
                            const nav_msgs::msg::Path& path) const {
      size_t nearest_idx = 0;
      double min_distance = std::numeric_limits<double>::max();
      
      RCLCPP_DEBUG(node_->get_logger(), "Finding interrupted index for target: (%.2f, %.2f)", 
                  target_pose.pose.position.x, target_pose.pose.position.y);
      
      for (size_t i = 0; i < path.poses.size(); ++i) {
          double distance = pointDistance(target_pose.pose.position, path.poses[i].pose.position);
          
          if (distance < min_distance) {
              min_distance = distance;
              nearest_idx = i;
          }
      }

      RCLCPP_INFO(node_->get_logger(), "Target index found at %zu (distance: %.3f)", 
                 nearest_idx, min_distance);
      
      return nearest_idx;
  }

  size_t findNextWaypoint(const geometry_msgs::msg::PoseStamped& robot_pose, 
                         const nav_msgs::msg::Path& path) const {
      size_t nearest_idx = 0;
      double min_distance = std::numeric_limits<double>::max();
      
      for (size_t i = 0; i < path.poses.size(); ++i) {
          double distance = pointDistance(robot_pose.pose.position, path.poses[i].pose.position);
          
          if (distance < min_distance) {
              min_distance = distance;
              nearest_idx = i;
          }
          
          if (min_distance < 0.1) {
              break;
          }
      }

      RCLCPP_DEBUG(node_->get_logger(), "Nearest waypoint index: %zu (distance: %.3f)", nearest_idx, min_distance);
      
      if (min_distance < 0.5 && nearest_idx + 1 < path.poses.size()) {
          RCLCPP_DEBUG(node_->get_logger(), "Close to waypoint, using next index: %zu", nearest_idx + 1);
          return nearest_idx + 1;
      }
      
      return nearest_idx;
  }

  double pointDistance(const geometry_msgs::msg::Point& p1, 
                     const geometry_msgs::msg::Point& p2) const {
      double dx = p1.x - p2.x;
      double dy = p1.y - p2.y;
      return std::sqrt(dx*dx + dy*dy);
  }
};

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<CoveragePathTrimmer>("CoveragePathTrimmer");
}