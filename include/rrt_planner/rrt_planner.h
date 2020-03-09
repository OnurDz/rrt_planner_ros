
#include <ros/ros.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>

#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>

#include <vector>
#include <random>
#include <boost/random.hpp>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

/** Debugging  */
#include <chrono>
#include <ctime>
/**            */

#ifndef RRTS_CPP
#define RRTS_CPP

namespace rrt_planner {
  /**
   * @brief a class interface for storing and manipulating map location data in rrt form
   */
  class RRT {
    public:
      /**
       * @brief struct for efficiently and simply storing coordinates at one object
       */
      struct Point {
        double x;
        double y;
      };
      
      /**
       * @brief class for storing and manipulating map coordinates
       */
      class Node {
        private:
          /** world x coordinate of node */
          double wx_;
          /** world y coordinate of node */
          double wy_;
          /** index in tree */
          int index_;
          /** index of parent in tree */
          int parent_;

        public:
          /**
           * @brief default constructor
           */
          Node() { parent_ = -1; }

          /**
           * @brief constructor that initializes with coordinates
           */
          Node(double wx, double wy) {
            parent_ = -1;
            setLocation(wx, wy);
          }

          /** 
           * @brief constructor that initializes with point struct
           */
          Node(Point p) {
            parent_ = -1;
            setLocation(p);
          }

          /**
           * @brief getter function for wx_
           */
          double getX() { return wx_; }

          /**
           * @brief getter function for wy_
           */
          double getY() { return wy_; }

          /**
           * @brief setter function for wx_
           */
          void setX(double wx) { wx_ = wx; }

          /**
           * @brief setter function for wy_
           */
          void setY(double wy) { wy_ = wy; }


          /**
           * @brief getter function for node index
           */
          int getIndex() { return index_; }

          /**
           * @brief getter function for parent index
           */
          int getParent() { return parent_; }

          /**
           * @brief setter function for node index
           */
          void setIndex(int index) { index_ = index; }

          /**
           * @brief getter function for parent index
           */
          void setParent(int pindex) { parent_ = pindex; }
          
          /**
           * @brief getter function for node location as point struct
           */
          Point getLocation() {
            Point p = { wx_, wy_ };
            return p;
          }
          
          /**
           * @brief setter function for node coordinates
           */
          void setLocation(double wx, double wy) {
            wx_ = wx;
            wy_ = wy;
          }

          /**
           * @brief setter function for node coordinates with point @param p
           */
          void setLocation(Point p) { setLocation(p.x, p.y); }

          /**
           * @brief return true if node has an assigned parent, false otherwise
           */
          bool hasParent() { return parent_ == -1; }

          /**
           * @brief override of comparison operator
           */
          void operator=(const Node &N) {
            wx_ = N.wx_;
            wy_ = N.wy_;
            parent_ = N.parent_;
          }
      };

      /**
       * @brief default constructor
       */
      RRT() {
        size_ = 0;
        tree_pub_ = tree_nh_.advertise<visualization_msgs::Marker>("tree_marker", 0);
      }

      /**
       * @brief return node object at @param index in tree
       */
      Node get(int index) { return list_.at(index); }

      /**
       * @brief return root node object of tree
       */
      Node getRoot() { return get(0); }

      /**
       * @brief return last added node object in tree
       */
      Node back() { return list_.back(); }

      /**
       * @brief return number of nodes in tree
       */
      int size() { return size_; }

      /**
       * @brief properly add node @param n to tree
       */
      void add(Node n) {
        n.setIndex(size_);
        list_.push_back(n);
        Node debug = list_[size_];
        size_++;
      }
      
      /**
       * @brief properly add node @param n to tree after setting its parent to @param pindex
       */
      void add(Node n, int pindex) {
        n.setParent(pindex);
        add(n);
      }

      /**
       * @brief properly add a node that contains information in @param p to tree
       */
      void add(Point p) {
        Node n(p);
        add(n);
      }

      /**
       * @brief visualize the tree in rviz
       */
      void visualize() {
        int marker_id = 0;
        visualization_msgs::Marker points;
        points.header.frame_id = "map";
        points.header.stamp = ros::Time::now();
        points.type = visualization_msgs::Marker::POINTS;
        points.action = visualization_msgs::Marker::ADD;
        points.id = marker_id++;
        points.color.r = 1.0;
        points.color.a = 0.5;
        points.pose.orientation.w = 1.0;
        points.scale.x = 0.07;
        points.scale.y = 0.07;

        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "map";
        line_strip.header.stamp = ros::Time::now();
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.id = marker_id++;
        line_strip.color.b = 1.0;
        line_strip.color.a = 0.5;
        line_strip.pose.orientation.w = 1.0;
        line_strip.scale.x = 0.06;

        geometry_msgs::Point p;
        for(int i = 0; i < size_; i++) {
          p.x = list_[i].getX();
          p.y = list_[i].getY();
          p.z = 0.0;

          points.points.push_back(p);
          tree_pub_.publish(points);
        }
        
      }

      void clear() {
          list_.clear();
          size_ = 0;
      }

    private:
      /** dynamic array to store nodes efficiently */
      std::vector<Node> list_;
      /** counter to store number of nodes in tree */
      int size_;

      ros::NodeHandle tree_nh_;
      ros::Publisher tree_pub_;
  };

  /**
   * @brief class ros calles to initialize, make and push plan
   */
  class RRTPlanner : public nav_core::BaseGlobalPlanner {
    public:
      /**
       * @brief default constructor
       */
      RRTPlanner();

      /**
       * @brief constructor
       * @param nh ROS NodeHandle object
       */
      RRTPlanner(ros::NodeHandle nh);

      /**
       * @brief constructor
       * @param name give a name to planner
       * @param costmap_ros pointer to costmap object ros gives to planner
       */
      RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
      /**
       * @brief initialize the planner
       * @param name give a name to planner
       * @param costmap_ros pointer to costmap object ros gives to planner
       */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
      /**
       * @brief overloaded makePlan function
       * @param start starting point of robot
       * @param goal goal point of robot
       * @param plan plan object to be filled and pushed
       */
      bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan
                   );
      /**
       * @brief return true if @param p is on an obstacle
       */
      bool obstacle(RRT::Point point);
      /**
       * @brief return true if @param parent is reachable by drawing a straight line from @param end
       */
      bool pathValid(RRT::Point start, RRT::Point end);

      /**
       * @brief calculate and return the euclidean distance between @param start and @param end
       */
      double distance(RRT::Point start, RRT::Point end);
      
      /**
       * @brief return true if @param point is in goal threshold
       */
      bool inGoalArea(RRT::Point point);
      
      /**
       * @brief return a random point inside map bounds
       */
      RRT::Point getRandom();
      
      /**
       * @brief generate and return PoseStamped message object according to data @param n holds
       */
      geometry_msgs::PoseStamped generatePoseStamped(RRT::Node n);

      /**
       * find nearest node to @param point and return its index in tree
       */
      int nearest(RRT::Point point);

      /**
       * publish visualization messages that contain node poses
       */

      void visualize(std::vector<geometry_msgs::PoseStamped> plan);
      double getLength(int index);
      RRT::Point addWithStep(RRT::Point p, int pindex);

    private:
      /** Costmap2DROS pointer object */
      costmap_2d::Costmap2DROS* costmap_ros_;
      /** Costmap2D pointer object */
      costmap_2d::Costmap2D* costmap_;
      /** WorldModel pointer object */
      base_local_planner::WorldModel* world_model_;

      /** tree object to hold vertices */
      rrt_planner::RRT* tree_;
      /** start node */
      RRT::Point start_;
      /** goal node */
      RRT::Point goal_;

      /** provide mutual exclusion between threads */
      boost::mutex mutex_;
      /** frame id of costmap */
      std::string frame_id_;
      /** name of the planner */
      std::string name_;

      /** bool to check if planner is initialized */
      bool initialized_;

      /** goal threshold */
      double goal_radius_;
      /** precision variable */
      double delta_;
      /** number of max iterations for a single sweep */
      int iteration_limit_;

      /** ROS NodeHandle object */
      ros::NodeHandle private_nh_;
      
      ros::NodeHandle vis_nh_;
      ros::Publisher vis_pub_;


      int mode_;
      double step_size_;
  };
}

#endif