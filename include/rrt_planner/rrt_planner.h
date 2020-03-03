
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
  class RRT {
    public:
      struct Point {
        double x;
        double y;
      };
      
      class Node {
        private:
          double wx_;
          double wy_;
          int index_;
          int parent_;

        public:
          Node() { parent_ = -1; }
          Node(double wx, double wy) {
            parent_ = -1;
            setLocation(wx, wy);
          }
          Node(Point p) {
            parent_ = -1;
            setLocation(p);
          }

          double getX() { return wx_; }
          double getY() { return wy_; }
          void setX(double wx) { wx_ = wx; }
          void setY(double wy) { wy_ = wy; }

          int getIndex() { return index_; }
          int getParent() { return parent_; }
          void setIndex(int index) { index_ = index; }
          void setParent(int pindex) { parent_ = pindex; }
          
          Point getLocation() {
            Point p = { wx_, wy_ };
            return p;
          }
          
          void setLocation(double wx, double wy) {
            wx_ = wx;
            wy_ = wy;
          }
          void setLocation(Point p) { setLocation(p.x, p.y); }

          bool hasParent() { return parent_ == -1; }

          void operator=(const Node &N) {
            wx_ = N.wx_;
            wy_ = N.wy_;
            parent_ = N.parent_;
          }
      };

      RRT() { size_ = 0; }

      Node get(int index) { return list_.at(index); }
      Node getRoot() { return get(0); }
      Node back() { return list_.back(); }
      int size() { return size_; }

      void add(Node n) {
        n.setIndex(size_);
        list_.push_back(n);
        //ROS_INFO("added (%.2f, %.2f) with parent %d", list_.at(size_).getX(), list_.at(size_).getY(), list_.at(size_).getParent());
        Node debug = list_[size_];
        //if(debug.getParent() != -1) {
          //ROS_INFO("parent location: (%.2f, %.2f)", get(debug.getParent()).getX(), get(debug.getParent()).getY());
        //}
        size_++;
      }
      void add(Node n, int pindex) {
        n.setParent(pindex);
        add(n);
      }
      void add(Point p) {
        Node n(p);
        add(n);
      }

      /** print tree for @debug **/
      void print() {
        printf("Root:\t(%.2f,\t%.2f)\n", list_[0].getX(), list_[0].getY());
        for(int i = 1; i < size_; i++) {
          Node current = get(i);
          printf("\nNode\t%d:\t(%.2f,\t%.2f)\n",   i, current.getX(), current.getY());
          printf("Parent\t%d:\t(%.2f,\t%.2f)\n", i, get(current.getParent()).getX(), get(current.getParent()).getY());
        }
      }

    private:
      std::vector<Node> list_;
      int size_;
  };

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
      void visualize(geometry_msgs::PoseStamped pose);


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

      /** Time object to set PoseStamped header time */
      ros::Time plan_time_;
      /** ROS NodeHandle object */
      ros::NodeHandle ros_nh_;

  };
}

#endif