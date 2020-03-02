#include "rrtgo/rrtgo.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(rrtgo::Planner, nav_core::BaseGlobalPlanner)

namespace rrtgo {
  Planner::Planner()
  : costmap_ros_(nullptr), initialized_(false) {}

  Planner::Planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : initialized_(false) {
    initialize(name, costmap_ros);
  }

  void Planner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    ROS_INFO("Initializing.");
    if(!initialized_) {
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();
      world_model_ = new base_local_planner::CostmapModel(*costmap_);
      ros::NodeHandle private_nh("~" + name);
      goal_radius_ = 0.3;
      delta_ = 0.01;
      iteration_limit_ = 500;
      frame_id_ = "map";
      initialized_ = true;
      ROS_INFO("Initialized planner %s", name);
    }
  }

  bool Planner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
    ROS_INFO("Making plan.");
    boost::mutex::scoped_lock(mutex_);

    start_ = { start.pose.position.x, start.pose.position.y };
    goal_ = { goal.pose.position.x, goal.pose.position.y };

    ROS_INFO("Start: %.2f, %.2f", start_.x, start_.y);
    ROS_INFO("Goal:  %.2f, %.2f", goal_.x, goal_.y);
    
    tree_ = new RRT();
    tree_->add(start_);

    ROS_INFO("Tree root: %.2f, %.2f", tree_->getRoot().getX(), tree_->getRoot().getY());
    
    int iterations = 0;
    bool goal_reached = false;

    while(!goal_reached && iterations < iteration_limit_) {
      RRT::Point random_point = getRandom();
      if(!obstacle(random_point)) {
        int nearest_index = nearest(random_point);
        RRT::Node nearest_node = tree_->get(nearest_index);
        if(pathValid(random_point, nearest_node.getLocation())) {
          tree_->add(random_point, nearest_index);
          if(inGoalArea(random_point)) {
            goal_reached = true;
            tree_->add(goal_, tree_->size() - 1);
          }
          iterations++;
        }
      }
    }

    if(!goal_reached) {
      ROS_WARN("Iteration limit reached. Path not found!");
      return false;
    }

    ROS_INFO("Tree completed.");
    ROS_INFO("Iterations: %d", iterations);
    tree_->print();


    int walk = tree_->size() - 1;
    std::vector<int> valid_list;
    while(walk != -1) {
      valid_list.push_back(walk);
      walk = tree_->get(walk).getParent();
    }
    



    plan.clear();

    for(int i = 0; i < valid_list.size(); i++) {
      plan.push_back(generatePoseStamped(tree_->get(valid_list[i])));
    }


    /**
     * @debug
     **/
    //for(int w = 0; w < plan.size(); w++) {
    //  printf("Waypoint\t%d:\t(%.2f, %.2f)\n", w, plan[w].pose.position.x, plan[w].pose.position.y);
    //}
    /***/

    return !plan.empty();
  
  }

  bool Planner::obstacle(RRT::Point point) {
    unsigned int mx, my;
    double wx = point.x;
    double wy = point.y;
    costmap_->worldToMap(wx, wy, mx, my);
    //ROS_INFO("Checking if (%.2f, %.2f) is obstacle.", wx, wy);
    if(costmap_->getCost(mx, my) >= costmap_2d::LETHAL_OBSTACLE) {
      //ROS_INFO("+Obstacle");
      return true;
    }
    //ROS_INFO("-Not Obstacle");
    return false;
  }


  bool Planner::pathValid(RRT::Point start, RRT::Point end) {
    double start_x, start_y, end_x, end_y;
    start_x = start.x;
    start_y = start.y;
    end_x = end.x;
    end_y = end.y;
    ROS_INFO("Checking if (%.2f, %.2f) ---> (%.2f, %.2f) path is valid.", start_x, start_y, end_x, end_y);

    double theta = atan2(start_y - end_y, start_x - end_x);

    RRT::Point current;
    double cur_x, cur_y;
    cur_x = start_x;
    cur_y = start_y;
    current.x = cur_x;
    current.y = cur_y;

    unsigned int mx, my;

    while(distance(current, end) > delta_) {
      //ROS_INFO("current: (%.2f, %.2f)", cur_x, cur_y);
      cur_x -= delta_ * cos(theta);
      cur_y -= delta_ * sin(theta);
      current.x = cur_x;
      current.y = cur_y;
      if(obstacle(current)) {
        return false;
      }
    }
    return true;
  }

  double Planner::distance(RRT::Point start, RRT::Point end) {
    double q1 = start.x;
    double q2 = start.y;
    double p1 = end.x;
    double p2 = end.y;
    return sqrt(pow((q1 - p1), 2) + pow((q2 - p2), 2));
  }

  bool Planner::inGoalArea(RRT::Point point) {
    return distance(point, goal_) <= goal_radius_;
  }

  RRT::Point Planner::getRandom() {
    ROS_INFO("Getting random point.");
    RRT::Point rand;
    std::random_device rd;
    std::mt19937 gen(rd());
    double world_x = costmap_->getSizeInMetersX();
    double world_y = costmap_->getSizeInMetersY();
    std::uniform_real_distribution<> x((-1) * world_x / 2, world_x / 2);
    std::uniform_real_distribution<> y((-1) * world_y / 2, world_y / 2);
    rand.x = x(gen);
    rand.y = y(gen);
    ROS_INFO("Random point: %.2f, %.2f", rand.x, rand.y);
    return rand;
  }

  geometry_msgs::PoseStamped Planner::generatePoseStamped(RRT::Node n) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = plan_time_;
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = n.getX();
    pose.pose.position.y = n.getY();
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    ROS_INFO("New PoseStamped: %.2f, %.2f", pose.pose.position.x, pose.pose.position.y);
    visualize(pose);
    
    return pose;
  }

  int Planner::nearest(RRT::Point point) {
    //ROS_INFO("Getting the closest node to (%.2f, %.2f)", point.x, point.y);
    double min_dist = std::numeric_limits<double>::infinity();
    int closest_index;
    int index;
    for(index = 0; index < tree_->size(); index++) {
      RRT::Point walk = tree_->get(index).getLocation();
      double dist = distance(point, walk);
      if(dist < min_dist) {
        min_dist = dist;
        closest_index = index;
      }
    }
    ROS_INFO("Tree size: %d", tree_->size());
    ROS_INFO("Closest index: %d", closest_index);
    return closest_index;
  }
  
  void Planner::visualize(geometry_msgs::PoseStamped pose) {
    /** Visualization */
    ros::NodeHandle vis_nh;
    ros::Publisher vis_pub = vis_nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "rrtgo";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose.pose;
    marker.scale.x = 1.0;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    vis_pub.publish(marker);
    printf("%.2f, %.2f\n", marker.pose.position.x, marker.pose.position.y);
    /**               */
  }

}