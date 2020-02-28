
#include <ros/ros.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>

#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>

#include <vector>
#include <random>
#include <boost/random.hpp>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#ifndef RRTS_CPP
#define RRTS_CPP

namespace rrtgo {
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
          Node* parent_;
          int parent_index_;

        public:
          Node() {
            init();
          }
          Node(double wx, double wy) {
            init();
            setLocation(wx, wy);
          }
          Node(Point p) {
            init();
            setLocation(p);
          }
          
          void init() {
            parent_ = NULL;
            parent_index_ = -1;
          }

          double getX() {
            return wx_;
          }
          double getY() {
            return wy_;
          }
          Point getLocation() {
            Point p = { wx_, wy_ };
            return p;
          }
          Node* getParent() {
            return parent_;
          }
          int getParentIndex() {
            return parent_index_;
          }
          
          void setX(double wx) {
            wx_ = wx;
          }
          void setY(double wy) {
            wy_ = wy;
          }
          void setLocation(Point p) {
            wx_ = p.x;
            wy_ = p.y;
          }
          void setLocation(double wx, double wy) {
            wx_ = wx;
            wy_ = wy;
          }
          void setParent(Node* parent) {
            parent_ = parent;
          }
          void setParentIndex(int pindex) {
            parent_index_ = pindex;
          }

          bool hasParent() {
            return parent_index_ == -1;
          }

          void operator=(const Node &N) {
            wx_ = N.wx_;
            wy_ = N.wy_;
            parent_ = N.parent_;
            parent_index_ = N.parent_index_;
          }
      };

      RRT(int capacity) {
        init(capacity);
      }
      RRT(int capacity, Node root) {
        init(capacity);
        add(root);
      }

      void init(int capacity) {
        capacity_ = capacity;
        list_ = new Node[capacity_];
        root_ = &list_[0];
      }

      Node* getRoot() {
        return root_;
      }
      Node* get(int index) {
        return &list_[index];
      }
      Node* getBack() {
        return &list_[size_ - 1];
      }
      int size() {
        return size_;
      }

      void add(Node n) {
        list_[size_] = n;
        size_++;
      }
      void add(Node n, Node* parent) {
        n.setParent(parent);
        add(n);
      }
      void add(Point p) {
        Node n(p);
        add(n);
      }
      void add(Point p, Node* parent) {
        Node n(p);
        add(n, parent);
      }

      void print() {
        printf("Root:\t(%.2f,\t%.2f)\n", root_->getX(), root_->getY());
        for(int i = 1; i < size_; i++) {
          printf("\nNode\t%d:\t(%.2f,\t%.2f)\nParent\t%d:\t(%.2f,\t%.2f)\n", 
                 i, get(i)->getX(),         get(i)->getY(),
                 i, get(i)->getParent()->getX(), get(i)->getParent()->getY());
        }
      }

    private:
      Node* list_;
      Node* root_;
      int size_;
      int capacity_;
  };

  class Planner : public nav_core::BaseGlobalPlanner {
    public:
      /**
       * @brief default constructor
       */
      Planner();
      /**
       * @brief constructor
       * @param name give a name to planner
       * @param costmap_ros pointer to costmap object ros gives to planner
       */
      Planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
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
      bool pathValid(RRT::Point start, RRT::Node end);

      double distance(RRT::Point start, RRT::Node end);
      /**
       * @brief return true if @param n is in goal threshold
       */
      bool inGoalArea(RRT::Point point);
      /**
       * @brief return a random point inside map bounds
       */
      RRT::Point getRandom();
      /**
       * @brief fill @param pos according to data @param n holds
       */
      geometry_msgs::PoseStamped generatePoseStamped(RRT::Node n);


      RRT::Node* closest(RRT::Point point);


    private:
      /** Costmap2DROS pointer object */
      costmap_2d::Costmap2DROS* costmap_ros_;
      /** Costmap2D pointer object */
      costmap_2d::Costmap2D* costmap_;
      /** WorldModel pointer object */
      base_local_planner::WorldModel* world_model_;

      /** tree object to hold vertices */
      rrtgo::RRT* tree_;
      /** start node */
      RRT::Node* start_;
      /** goal node */
      RRT::Node* goal_;

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

  };
}

#endif