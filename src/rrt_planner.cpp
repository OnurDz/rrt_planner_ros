#include <pluginlib/class_list_macros.h>
#include "rrt_planner/rrt_planner.h"

PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlanner, nav_core::BaseGlobalPlanner)

namespace rrt_planner {
  RRTPlanner::RRTPlanner()
  : costmap_ros_(nullptr), initialized_(false), private_nh_("~") {}

  RRTPlanner::RRTPlanner(ros::NodeHandle nh)
  : costmap_ros_(nullptr), initialized_(false), private_nh_("~") {
    private_nh_ = nh;
  }

  RRTPlanner::RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : initialized_(false), private_nh_("~") {
    initialize(name, costmap_ros);
  }

  void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    ROS_INFO("Initializing.");
    if(!initialized_) {
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();
      world_model_ = new base_local_planner::CostmapModel(*costmap_);
      tree_ = new RRT();
      private_nh_.param("/move_base/mode", mode_, 1);
      private_nh_.param("/move_base/goal_radius", goal_radius_, 0.2);
      private_nh_.param("/move_base/step_size", step_size_, 0.2);
      private_nh_.param("/move_base/delta", delta_, 0.08);
      private_nh_.param("/move_base/iteration_limit", iteration_limit_, 750);
      ROS_INFO("Mode: %d", mode_);
      ROS_INFO("Goal radius: %.2f", goal_radius_);
      ROS_INFO("Step size: %.2f", step_size_);
      ROS_INFO("Delta: %.3f", delta_);
      ROS_INFO("Iteration limit: %d", iteration_limit_);


      frame_id_ = costmap_ros_->getGlobalFrameID();
      initialized_ = true;
      ROS_INFO("Initialized planner %s", name.c_str());


      vis_pub_ = vis_nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    }
  }

  bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
    
    /** Determine proficency and efficiency */
    auto st = std::chrono::system_clock::now();
    /**                                     */

    ROS_INFO("Making plan.");
    boost::mutex::scoped_lock(mutex_);
    tree_->clear();
    start_ = { start.pose.position.x, start.pose.position.y };
    goal_ = { goal.pose.position.x, goal.pose.position.y };

    if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()) {
      ROS_ERROR("This planner will only accept goals in the %s frame,"
                "the goal was sent to the %s frame.",
                costmap_ros_->getGlobalFrameID().c_str(),
                goal.header.frame_id.c_str());
      return false;
    }

    ROS_INFO("Start: %.2f, %.2f", start_.x, start_.y);
    ROS_INFO("Goal:  %.2f, %.2f", goal_.x, goal_.y);

    tree_->add(start_);

    ROS_INFO("Tree root: %.2f, %.2f", tree_->getRoot().getX(), tree_->getRoot().getY());
    
    int iterations = 0;
    bool goal_reached = false;
    std::vector<int> valid_list;

    while(iterations < iteration_limit_) {
      RRT::Point random_point = getRandom();
      if(!obstacle(random_point)) {
        int nearest_index = nearest(random_point);
        RRT::Point nearest_loc = tree_->get(nearest_index).getLocation();
        if(pathValid(random_point, nearest_loc)) {
          random_point = addWithStep(random_point, nearest_index);
          if(inGoalArea(random_point)) {
            goal_reached = true;
            if(mode_ == 0) {
              tree_->add(goal_, tree_->size() - 1);
              break;
            } else {
              valid_list.push_back(tree_->size() - 1);
            }
          }
          iterations++;
        }
      }
    }
    
    tree_->visualize();

    if(!goal_reached) {
      ROS_WARN("Iteration limit of %d reached. Path not found!", iteration_limit_);
      return false;
    }
    

    ROS_INFO("Tree completed.");
    ROS_INFO("Iterations: %d", iterations);
    ROS_INFO("Tree size: %d", tree_->size());


    if(mode_ == 0) {
      int walk = tree_->size() - 1;
      while(walk != -1) {
        valid_list.push_back(walk);
        walk = tree_->get(walk).getParent();
      }

    } else {
      double length = -1;
      double min_length = std::numeric_limits<double>::infinity();
      int candidate = -1;
      for(int i = 0; i < valid_list.size(); i++) {
        length = getLength(valid_list[i]);
        if(min_length > length) {
          min_length = length;
          candidate = valid_list[i];
        }
      }
      plan.clear();
      valid_list.clear();
      tree_->add(goal_, candidate);
      int walk = tree_->size() - 1;;
      while(walk != -1) {
        valid_list.push_back(walk);
        walk = tree_->get(walk).getParent();
      }
      
    }

    plan.clear();

    for(int i = 0; i < valid_list.size(); i++) {
      plan.push_back(generatePoseStamped(tree_->get(valid_list[i])));
    }

    visualize(plan);

    /** Elapsed time calculation for debugging */
    auto en = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = en - st;
    ROS_INFO("ELAPSED TIME: %f seconds", elapsed_seconds.count());
    /**                                        */

    return !plan.empty();
  
  }

  bool RRTPlanner::obstacle(RRT::Point point) {
    unsigned int mx, my;
    double wx = point.x;
    double wy = point.y;
    costmap_->worldToMap(wx, wy, mx, my);
    unsigned char cost = costmap_->getCost(mx, my);
    if(cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
      return true;
    }
    return false;
  }


  bool RRTPlanner::pathValid(RRT::Point start, RRT::Point end) {
    double start_x, start_y, end_x, end_y;
    start_x = start.x;
    start_y = start.y;
    end_x = end.x;
    end_y = end.y;

    float theta = atan2(start_y - end_y, start_x - end_x);
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);

    RRT::Point current;
    double cur_x, cur_y;
    cur_x = start_x;
    cur_y = start_y;
    current.x = cur_x;
    current.y = cur_y;

    unsigned int mx, my;

    while(distance(current, end) > delta_) {
      cur_x -= delta_ * cos_theta;
      cur_y -= delta_ * sin_theta;
      current.x = cur_x;
      current.y = cur_y;
      if(obstacle(current)) {
        return false;
      }
    }
    return true;
  }

  RRT::Point RRTPlanner::addWithStep(RRT::Point p, int pindex) {
    RRT::Node parent = tree_->get(pindex);
    double rx, ry, px, py;
    rx = p.x;
    ry = p.y;
    px = parent.getX();
    py = parent.getY();

    double theta = atan2(ry - py, rx - px);

    RRT::Point new_point;
    new_point.x = px + step_size_ * cos(theta);
    new_point.y = py + step_size_ * sin(theta);

    tree_->add(new_point, pindex);

    return new_point;
  }

  double RRTPlanner::distance(RRT::Point start, RRT::Point end) {
    double q1 = start.x;
    double q2 = start.y;
    double p1 = end.x;
    double p2 = end.y;
    return sqrt(pow((q1 - p1), 2) + pow((q2 - p2), 2));
  }

  bool RRTPlanner::inGoalArea(RRT::Point point) {
    return distance(point, goal_) <= goal_radius_;
  }

  RRT::Point RRTPlanner::getRandom() {
    RRT::Point rand;
    std::random_device rd;
    std::mt19937 gen(rd());
    double world_x = costmap_->getSizeInMetersX();
    double world_y = costmap_->getSizeInMetersY();
    std::uniform_real_distribution<> x((-1) * world_x / 2, world_x / 2);
    std::uniform_real_distribution<> y((-1) * world_y / 2, world_y / 2);
    rand.x = x(gen);
    rand.y = y(gen);
    return rand;
  }

  geometry_msgs::PoseStamped RRTPlanner::generatePoseStamped(RRT::Node n) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = n.getX();
    pose.pose.position.y = n.getY();
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    ROS_INFO("New PoseStamped: %.2f, %.2f", pose.pose.position.x, pose.pose.position.y);
    
    return pose;
  }

  int RRTPlanner::nearest(RRT::Point point) {
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
    return closest_index;
  }

  void RRTPlanner::visualize(std::vector<geometry_msgs::PoseStamped> plan) {
    int marker_id = 0;
    visualization_msgs::Marker points;
    points.header.frame_id = frame_id_;
    points.header.stamp = ros::Time::now();
    points.type = visualization_msgs::Marker::POINTS;
    points.action = visualization_msgs::Marker::ADD;
    points.id = marker_id++;
    points.color.r = 1.0;
    points.color.a = 1.0;
    points.pose.orientation.w = 1.0;
    points.scale.x = 0.075;
    points.scale.y = 0.075;

    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = frame_id_;
    line_strip.header.stamp = ros::Time::now();
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.id = marker_id++;
    line_strip.color.g = 1.0;
    line_strip.color.a = 1.0;
    line_strip.pose.orientation.w = 1.0;
    line_strip.scale.x = 0.05;

    geometry_msgs::Point p;
    for(int i = 0; i < plan.size(); i++) {
      p.x = plan[i].pose.position.x;
      p.y = plan[i].pose.position.y;
      p.z = plan[i].pose.position.z;

      points.points.push_back(p);
      line_strip.points.push_back(p);
    }
    vis_pub_.publish(points);
    vis_pub_.publish(line_strip);
    ros::Duration(0.5).sleep();
  }

  double RRTPlanner::getLength(int index) {
    RRT::Node walk, parent;
    int len = 0;
    walk = tree_->get(index);
    while(walk.getParent() >= 0) {
      parent = tree_->get(walk.getParent());
      len++;
      walk = parent;
    }
    return len;
  }
  
}