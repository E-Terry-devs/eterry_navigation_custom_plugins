#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <chrono>
#include <mutex>
#include <thread>
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

class ResumePathTrimmer : public BT::SyncActionNode
{
public:
  ResumePathTrimmer(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("resume_path_trimmer_node");
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Debug publishers: visualize the next upcoming waypoints in RViz
    debug_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
      "resume_path_trimmer/trimmed_path", 5);
    debug_markers_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "resume_path_trimmer/waypoints", 5);

    // Charger les rows au démarrage
    rows_ = parseFieldRows("/home/hedi/eterry_simulation/src/eterry_sim_stack/simulation_navigation/maps/output.xml");

    RCLCPP_INFO(node_->get_logger(), "ResumePathTrimmer initialized with %zu rows", rows_.size());
    for (const auto& row : rows_) {
        RCLCPP_DEBUG(node_->get_logger(), "  %s", row.toString().c_str());
    }

    // ResumePathTrimmer only ticks once per "resume" episode (BT Sequence semantics),
    // so the debug view needs its own live loop, decoupled from BT ticking, to keep
    // sliding forward as the robot advances. Spin node_ on a dedicated thread so this
    // wall timer actually fires.
    executor_.add_node(node_);
    spin_thread_ = std::thread([this]() { executor_.spin(); });
    debug_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&ResumePathTrimmer::publishDebugWindow, this));
  }

  ~ResumePathTrimmer() override
  {
    executor_.cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
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
  RCLCPP_INFO(node_->get_logger(), "=== ResumePathTrimmer START ===");
  
  // -----------------------------------------------------------------
  // 1. Récupérer le chemin d'entrée
  // -----------------------------------------------------------------
  nav_msgs::msg::Path input_path;
  auto res_nav_path = getInput("nav_path", input_path);
  if (!res_nav_path) {
    RCLCPP_ERROR(node_->get_logger(), "Missing required input [nav_path]");
    setOutput("trimmed_path", createMinimalPathFromRobotPose());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "Input path has %zu poses", input_path.poses.size());
  if (input_path.poses.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Input path is empty");
    setOutput("trimmed_path", createMinimalPathFromRobotPose());
    return BT::NodeStatus::FAILURE;
  }

  // -----------------------------------------------------------------
  // 2. Récupérer la pose actuelle du robot
  // -----------------------------------------------------------------
  geometry_msgs::msg::PoseStamped current_robot_pose;
  if (!getCurrentRobotPose(current_robot_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get current robot pose from TF");
    setOutput("trimmed_path", input_path);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "Current robot pose: (%.2f, %.2f)", 
              current_robot_pose.pose.position.x, current_robot_pose.pose.position.y);

  // -----------------------------------------------------------------
  // 3. Trouver l'index de reprise
  // -----------------------------------------------------------------
  size_t target_idx = 0;
  bool is_within_path = isRobotInCoveragePath(current_robot_pose, input_path);
  RCLCPP_INFO(node_->get_logger(), "Robot is within coverage path: %s", is_within_path ? "YES" : "NO");
  
  if (is_within_path) {
    target_idx = findNextWaypoint(current_robot_pose, input_path);
    RCLCPP_INFO(node_->get_logger(), "Robot on path, next waypoint index: %zu", target_idx);
  } else {
    geometry_msgs::msg::PoseStamped target_pose;
    auto res_interrupted = getInput("interrupted_pose", target_pose);
    if (!res_interrupted) {
      target_pose = current_robot_pose;
      RCLCPP_WARN(node_->get_logger(), "No interrupted_pose provided, using current robot pose as target");
    }
    target_idx = findInterruptedIndex(target_pose, input_path);
    RCLCPP_INFO(node_->get_logger(), "Robot off path, resuming from index: %zu", target_idx);
  }
  
  // -----------------------------------------------------------------
  // 4. Construire le chemin AVEC interpolation (comme CoveragePathTrimmer)
  // -----------------------------------------------------------------
  nav_msgs::msg::Path trimmed_path;
  trimmed_path.header = input_path.header;
  bool success = true;
  
  if (target_idx >= input_path.poses.size()) {
    RCLCPP_WARN(node_->get_logger(), "Target index out of range, creating minimal path");
    trimmed_path = createPathFromRobotPose(current_robot_pose);
    success = false;
  } else {
    // Utiliser generateSafePathToRow comme dans la version qui fonctionne
    trimmed_path = generateSafePathToRow(current_robot_pose, 
                                        input_path.poses[target_idx], 
                                        input_path, 
                                        target_idx);
    RCLCPP_INFO(node_->get_logger(), "Generated safe path with %zu points from index %zu", 
                trimmed_path.poses.size(), target_idx);
    success = !trimmed_path.poses.empty();
  }

  // -----------------------------------------------------------------
  // 5. Validation finale
  // -----------------------------------------------------------------
  if (trimmed_path.poses.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Generated path is empty, falling back to minimal path (robot pose)");
    trimmed_path = createPathFromRobotPose(current_robot_pose);
    success = false;
  }

  // Forcer les frames
  trimmed_path.header.frame_id = "map";
  trimmed_path.header.stamp = node_->now();
  for (auto& pose : trimmed_path.poses) {
    pose.header.frame_id = "map";
    pose.header.stamp = node_->now();
  }

  // -----------------------------------------------------------------
  // 6. Publier (blackboard BT = chemin complet)
  // -----------------------------------------------------------------
  setOutput("trimmed_path", trimmed_path);

  // Le timer de debug (publishDebugWindow) republie en continu les 10
  // prochains points à partir de ce chemin, glissant au fur et à mesure
  // que le robot avance -- indépendant du tick du Sequence BT.
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    latest_full_path_ = trimmed_path;
  }

  RCLCPP_INFO(node_->get_logger(), "=== ResumePathTrimmer END ===");
  RCLCPP_INFO(node_->get_logger(), "Final trimmed path: %zu waypoints", trimmed_path.poses.size());
  if (!trimmed_path.poses.empty()) {
    RCLCPP_INFO(node_->get_logger(), "First point: (%.2f, %.2f) distance from robot: %.3f", 
                trimmed_path.poses[0].pose.position.x,
                trimmed_path.poses[0].pose.position.y,
                pointDistance(current_robot_pose.pose.position, trimmed_path.poses[0].pose.position));
  }

  return success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
private:
  std::vector<Row> rows_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_markers_pub_;
  rclcpp::TimerBase::SharedPtr debug_timer_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  std::mutex path_mutex_;
  nav_msgs::msg::Path latest_full_path_;

  // Constants for safety
  const double WAYPOINT_SPACING = 0.5;
  const double PROXIMITY_THRESHOLD = 1.0;
  // Debug viz only: how many upcoming waypoints to publish for RViz (full path still goes to FollowPath)
  const size_t DEBUG_MAX_WAYPOINTS = 10;

  // -----------------------------------------------------------------
  // Debug : republier en continu (200ms) les DEBUG_MAX_WAYPOINTS prochains
  // points à partir de la pose actuelle du robot, sur le dernier chemin connu.
  // Tourne sur son propre thread/timer, indépendant du tick BT.
  // -----------------------------------------------------------------
  void publishDebugWindow()
  {
    nav_msgs::msg::Path path_copy;
    {
      std::lock_guard<std::mutex> lock(path_mutex_);
      path_copy = latest_full_path_;
    }
    if (path_copy.poses.empty()) {
      return;
    }

    geometry_msgs::msg::PoseStamped robot_pose;
    if (!getCurrentRobotPose(robot_pose)) {
      return;
    }

    size_t nearest_idx = findNextWaypoint(robot_pose, path_copy);
    size_t end_idx = std::min(path_copy.poses.size(), nearest_idx + DEBUG_MAX_WAYPOINTS);

    nav_msgs::msg::Path debug_path;
    debug_path.header = path_copy.header;
    debug_path.header.frame_id = "map";
    debug_path.header.stamp = node_->now();
    debug_path.poses.assign(
      path_copy.poses.begin() + nearest_idx,
      path_copy.poses.begin() + end_idx);

    debug_path_pub_->publish(debug_path);
    publishWaypointMarkers(debug_path);
  }

  // -----------------------------------------------------------------
  // Debug : publier chaque waypoint du chemin comme une sphère numérotée
  // (visible dans RViz via un display "MarkerArray" sur le topic
  // resume_path_trimmer/waypoints, pratique pour quelqu'un qui ne connaît
  // pas le robot)
  // -----------------------------------------------------------------
  void publishWaypointMarkers(const nav_msgs::msg::Path& path) const
  {
    visualization_msgs::msg::MarkerArray markers;

    // Effacer les markers du tick précédent avant de republier les nouveaux
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = "map";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);

    for (size_t i = 0; i < path.poses.size(); ++i) {
      visualization_msgs::msg::Marker sphere;
      sphere.header.frame_id = "map";
      sphere.header.stamp = node_->now();
      sphere.ns = "resume_path_trimmer_waypoints";
      sphere.id = static_cast<int>(i);
      sphere.type = visualization_msgs::msg::Marker::SPHERE;
      sphere.action = visualization_msgs::msg::Marker::ADD;
      sphere.pose = path.poses[i].pose;
      sphere.scale.x = 0.15;
      sphere.scale.y = 0.15;
      sphere.scale.z = 0.15;
      sphere.color.r = (i == 0) ? 1.0f : 0.0f;
      sphere.color.g = (i == 0) ? 0.0f : 1.0f;
      sphere.color.b = 0.0f;
      sphere.color.a = 1.0f;
      markers.markers.push_back(sphere);

      visualization_msgs::msg::Marker label;
      label.header = sphere.header;
      label.ns = "resume_path_trimmer_waypoint_labels";
      label.id = static_cast<int>(i);
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose = path.poses[i].pose;
      label.pose.position.z += 0.3;
      label.scale.z = 0.2;
      label.color.r = 1.0f;
      label.color.g = 1.0f;
      label.color.b = 1.0f;
      label.color.a = 1.0f;
      label.text = std::to_string(i);
      markers.markers.push_back(label);
    }

    debug_markers_pub_->publish(markers);
  }

  // -----------------------------------------------------------------
  // Helper : créer un chemin contenant uniquement la pose robot
  // -----------------------------------------------------------------
  nav_msgs::msg::Path createPathFromRobotPose(const geometry_msgs::msg::PoseStamped& robot_pose) const
  {
    nav_msgs::msg::Path path;
    path.header = robot_pose.header;
    path.poses.push_back(robot_pose);
    return path;
  }

  // -----------------------------------------------------------------
  // Helper : créer un chemin minimal depuis la pose robot actuelle
  // -----------------------------------------------------------------
  nav_msgs::msg::Path createMinimalPathFromRobotPose()
  {
    nav_msgs::msg::Path path;
    geometry_msgs::msg::PoseStamped robot_pose;
    if (getCurrentRobotPose(robot_pose)) {
      path.header = robot_pose.header;
      path.poses.push_back(robot_pose);
    } else {
      path.header.frame_id = "map";
      path.header.stamp = node_->now();
    }
    return path;
  }

  // -----------------------------------------------------------------
  // Récupération de la pose robot via TF
  // -----------------------------------------------------------------
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

  // -----------------------------------------------------------------
  // Génération d'un chemin sécurisé vers le début de la row
  // -----------------------------------------------------------------
  nav_msgs::msg::Path generateSafePathToRow(const geometry_msgs::msg::PoseStamped& start_pose,
                                          const geometry_msgs::msg::PoseStamped& row_start_pose,
                                          const nav_msgs::msg::Path& original_path,
                                          size_t row_start_idx) const {
    RCLCPP_DEBUG(node_->get_logger(), "generateSafePathToRow called");
    nav_msgs::msg::Path safe_path;
    safe_path.header = original_path.header;
    
    safe_path.poses.push_back(start_pose);
    
    double total_distance = pointDistance(start_pose.pose.position, row_start_pose.pose.position);
    
    if (total_distance > WAYPOINT_SPACING) {
      int num_intermediate_points = std::max(1, static_cast<int>(total_distance / WAYPOINT_SPACING));
      for (int i = 1; i <= num_intermediate_points; ++i) {
        double ratio = static_cast<double>(i) / (num_intermediate_points + 1);
        geometry_msgs::msg::PoseStamped intermediate_pose = start_pose;
        intermediate_pose.pose.position.x = start_pose.pose.position.x + 
                                           (row_start_pose.pose.position.x - start_pose.pose.position.x) * ratio;
        intermediate_pose.pose.position.y = start_pose.pose.position.y + 
                                           (row_start_pose.pose.position.y - start_pose.pose.position.y) * ratio;
        if (i == num_intermediate_points) {
          intermediate_pose.pose.orientation = row_start_pose.pose.orientation;
        }
        safe_path.poses.push_back(intermediate_pose);
      }
    }
    
    safe_path.poses.push_back(row_start_pose);
    
    if (row_start_idx + 1 < original_path.poses.size()) {
      safe_path.poses.insert(safe_path.poses.end(),
                           original_path.poses.begin() + row_start_idx + 1,
                           original_path.poses.end());
    }
    
    return safe_path;
  }

  // -----------------------------------------------------------------
  // Parser XML (inchangé)
  // -----------------------------------------------------------------
  Point parsePoint(const std::string& pointStr) const {
      Point p;
      std::string cleaned = pointStr;
      std::replace(cleaned.begin(), cleaned.end(), ',', ' ');
      std::istringstream iss(cleaned);
      iss >> p.x >> p.y;
      return p;
  }

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
          if (rowElem->QueryIntAttribute("id", &rowId) != tinyxml2::XML_SUCCESS) continue;
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
              if (!token.empty()) points.push_back(token);
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

  geometry_msgs::msg::Point toPointMsg(const Point& p) const {
      geometry_msgs::msg::Point point_msg;
      point_msg.x = p.x;
      point_msg.y = p.y;
      point_msg.z = 0.0;
      return point_msg;
  }

  // -----------------------------------------------------------------
  // Détection de row (inchangé)
  // -----------------------------------------------------------------
  int findClosestRow(const geometry_msgs::msg::PoseStamped& pose) const {
      if (rows_.empty()) return -1;
      int closest_row_id = -1;
      double min_distance = std::numeric_limits<double>::max();
      for (const auto& row : rows_) {
          double dist_to_start = pointDistance(pose.pose.position, toPointMsg(row.start));
          double dist_to_end = pointDistance(pose.pose.position, toPointMsg(row.end));
          double min_dist_to_row = std::min(dist_to_start, dist_to_end);
          if (min_dist_to_row < min_distance) {
              min_distance = min_dist_to_row;
              closest_row_id = row.id;
          }
      }
      return closest_row_id;
  }

  size_t findRowStartInPath(int row_id, const nav_msgs::msg::Path& path) const {
      if (rows_.empty() || path.poses.empty()) return 0;
      Row target_row;
      bool found = false;
      for (const auto& row : rows_) {
          if (row.id == row_id) { target_row = row; found = true; break; }
      }
      if (!found) return 0;
      Point row_start_point = target_row.getBottomPoint();
      size_t closest_idx = 0;
      double min_distance = std::numeric_limits<double>::max();
      for (size_t i = 0; i < path.poses.size(); ++i) {
          double distance = pointDistance(path.poses[i].pose.position, toPointMsg(row_start_point));
          if (distance < min_distance) {
              min_distance = distance;
              closest_idx = i;
          }
          if (distance < 0.5) break;
      }
      return closest_idx;
  }

  size_t findRowStart(size_t target_idx, const nav_msgs::msg::Path& path) const {
      if (rows_.empty()) return findRowStartByAngle(target_idx, path);
      if (target_idx >= path.poses.size()) target_idx = path.poses.size() - 1;
      geometry_msgs::msg::PoseStamped target_pose = path.poses[target_idx];
      int closest_row_id = findClosestRow(target_pose);
      if (closest_row_id == -1) return findRowStartByAngle(target_idx, path);
      return findRowStartInPath(closest_row_id, path);
  }

  size_t findRowStartByAngle(size_t target_idx, const nav_msgs::msg::Path& path) const {
      double angle_threshold = 0.5;
      size_t start_index = std::min(target_idx, path.poses.size() - 1);
      for (size_t i = start_index; i >= 2; --i) {
          double dx1 = path.poses[i].pose.position.x - path.poses[i-1].pose.position.x;
          double dy1 = path.poses[i].pose.position.y - path.poses[i-1].pose.position.y;
          double dx2 = path.poses[i-1].pose.position.x - path.poses[i-2].pose.position.x;
          double dy2 = path.poses[i-1].pose.position.y - path.poses[i-2].pose.position.y;
          double angle1 = atan2(dy1, dx1);
          double angle2 = atan2(dy2, dx2);
          double angle_diff = fabs(angle1 - angle2);
          if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;
          if (angle_diff > angle_threshold) return i;
          double dist = pointDistance(path.poses[i].pose.position, path.poses[i-1].pose.position);
          if (dist > 3.0) return i;
      }
      return (target_idx > 0) ? target_idx : 0;
  }

  bool isRobotInCoveragePath(const geometry_msgs::msg::PoseStamped& robot_pose, 
                           const nav_msgs::msg::Path& path) const {
      for (const auto& pose : path.poses) {
          if (pointDistance(robot_pose.pose.position, pose.pose.position) < PROXIMITY_THRESHOLD)
              return true;
      }
      return false;
  }

  size_t findInterruptedIndex(const geometry_msgs::msg::PoseStamped& target_pose,
                            const nav_msgs::msg::Path& path) const {
      size_t nearest_idx = 0;
      double min_distance = std::numeric_limits<double>::max();
      for (size_t i = 0; i < path.poses.size(); ++i) {
          double distance = pointDistance(target_pose.pose.position, path.poses[i].pose.position);
          if (distance < min_distance) {
              min_distance = distance;
              nearest_idx = i;
          }
      }
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
          if (min_distance < 0.1) break;
      }
      if (min_distance < 0.5 && nearest_idx + 1 < path.poses.size())
          return nearest_idx + 1;
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
  factory.registerNodeType<ResumePathTrimmer>("ResumePathTrimmer");
}