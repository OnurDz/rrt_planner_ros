#include <pluginlib/class_list_macros.h>
#include "rrt_planner/rrt_planner.h"

PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlanner, nav_core::BaseGlobalPlanner)

namespace rrt_planner {
  RRTPlanner::RRTPlanner()
  : costmap_ros_(nullptr), initialized_(false) {}

  RRTPlanner::RRTPlanner(ros::NodeHandle nh)
  : costmap_ros_(nullptr), initialized_(false) {
    ros_nh_ = nh;
  }

  RRTPlanner::RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : initialized_(false) {
    initialize(name, costmap_ros);
  }

  void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    ROS_INFO("Initializing.");
    if(!initialized_) {
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();
      world_model_ = new base_local_planner::CostmapModel(*costmap_);
      ros::NodeHandle private_nh("~/" + name);
      private_nh.param("goal_radius", goal_radius_, 0.3);
      private_nh.param("delta", delta_, 0.005);
      private_nh.param("iteration_limit", iteration_limit_, 500);
      frame_id_ = "map";
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
      ROS_WARN("Iteration limit of %d reached. Path not found!", iteration_limit_);
      return false;
    }

    ROS_INFO("Tree completed.");
    ROS_INFO("Iterations: %d", iterations);
    ROS_INFO("Tree size: %d", tree_->size());
    //tree_->print();


    int walk = tree_->size() - 1;
    std::vector<int> valid_list;
    while(walk != -1) {
      valid_list.push_back(walk);
      walk = tree_->get(walk).getParent();
    }
    



    plan.clear();

    plan_time_ = ros::Time::now();
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
    //ROS_INFO("Checking if (%.2f, %.2f) is obstacle.", wx, wy);
    unsigned char cost = costmap_->getCost(mx, my);
    if(cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE || cost == costmap_2d::NO_INFORMATION) {
      //ROS_INFO("+Obstacle");
      return true;
    }
    //ROS_INFO("-Not Obstacle");
    return false;
  }


  bool RRTPlanner::pathValid(RRT::Point start, RRT::Point end) {
    double start_x, start_y, end_x, end_y;
    start_x = start.x;
    start_y = start.y;
    end_x = end.x;
    end_y = end.y;
    //ROS_INFO("Checking if (%.2f, %.2f) ---> (%.2f, %.2f) path is valid.", start_x, start_y, end_x, end_y);

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
    //ROS_INFO("Getting random point.");
    RRT::Point rand;
    std::random_device rd;
    std::mt19937 gen(rd());
    double world_x = costmap_->getSizeInMetersX();
    double world_y = costmap_->getSizeInMetersY();
    std::uniform_real_distribution<> x((-1) * world_x / 2, world_x / 2);
    std::uniform_real_distribution<> y((-1) * world_y / 2, world_y / 2);
    rand.x = x(gen);
    rand.y = y(gen);
    //ROS_INFO("Random point: %.2f, %.2f", rand.x, rand.y);
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
    pose.header.stamp = plan_time_;
    ROS_INFO("New PoseStamped: %.2f, %.2f", pose.pose.position.x, pose.pose.position.y);
    
    return pose;
  }

  int RRTPlanner::nearest(RRT::Point point) {
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
    //ROS_INFO("Tree size: %d", tree_->size());
    //ROS_INFO("Closest index: %d", closest_index);
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
  }
  

}