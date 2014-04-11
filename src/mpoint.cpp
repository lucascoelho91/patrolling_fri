#include <iostream>
#include <list>
#include <vector>
#include <stdio.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf/tf.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Controller
{
  private: 

    std::list< std::vector<double> > listGoals;

    ros::Publisher movebasePub;  
    ros::Subscriber movebaseStatus;

    ros::NodeHandle *node;
    
    MoveBaseClient ac;
    move_base_msgs::MoveBaseGoal currentGoal;

  public:

    void moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
    {
      if(msg->status_list[0].status == 3)
      {
        ROS_INFO("Goal id: %s\n", msg->status_list[0].goal_id.id.c_str());
        if (!this->popListToGoal())
        {
          printf("We reached the end of the list of goals! Nothing to do anymore.\n");
          exit(0);
        }
        else
        {
          this->publishGoal();
          this->waitResultNavigation();
        }
      }
    }

    void publishGoal()
    {
      ac.sendGoal(currentGoal);
    }

    void waitResultNavigation()
    {
      ac.waitForResult();
    }

    actionlib::SimpleClientGoalState getNavigationState()
    {
      return ac.getState();
    }

    void setCurrentGoal(double x, double y, double theta=0)
    {
      currentGoal.target_pose.header.frame_id = "/map";
      currentGoal.target_pose.pose.position.x = x;
      currentGoal.target_pose.pose.position.y = y;
      currentGoal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
    }

    bool popListToGoal()
    {
      std::vector<double> p2(3);
      if(listGoals.size() <= 0)
        return 0;
      else
      {
        p2 = listGoals.front();
        setCurrentGoal(p2[0], p2[1], p2[2]);
        listGoals.pop_front();
        return 1;
      }
    }

    void setMoveBasePublisher(std::string topic, int buffersize)
    {
      movebasePub = node->advertise<geometry_msgs::PoseStamped>(topic, buffersize);
    }

    void setMoveBaseStatusSubscriber(std::string topic, int buffersize)
    {
      movebaseStatus =  node->subscribe(topic, buffersize, &Controller::moveBaseStatusCallback, this);
    }

    bool readGoalsFile(std::string fileName)
    {
      FILE * pFile;
      pFile = fopen(fileName.c_str(), "r");
      if(pFile==NULL){
        printf("File not found.\n");
        exit(1);
      }

      std::vector<double> pin(3);
      for (;!feof(pFile);)
      {
        fscanf(pFile, "%lf %lf %lf", &pin[0], &pin[1], &pin[2]);
        listGoals.push_back(pin);
      }
      fclose(pFile);
    }

    bool addGoalToList(double x, double y, double theta=0)
    {
      std::vector<double> pin(3);
      pin[0] = x;
      pin[1] = y;
      pin[2] = theta;
      try {
        listGoals.push_back(pin);
      }
      catch (std::bad_alloc& ba)
      {
        printf("Error while trying to add to the list.\n");
        return 0;
      }
      return 1;
    }

    void waitForMoveBaseServer()
    {
      ac.waitForServer(ros::Duration(5.0));
    }

    Controller(ros::NodeHandle *n, std::string base_name="move_base")
    : ac(base_name, true)   
    {
      node = n;
    }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "navPoints");
  ros::NodeHandle node;

  geometry_msgs::PoseStamped posegoal;

  ROS_INFO("Init");
  Controller rosControl(&node);
  ROS_INFO("Created controller");

  rosControl.setMoveBaseStatusSubscriber("move_base/status", 1);
  rosControl.setMoveBasePublisher("/move_base_simple/goal", 1);
  ROS_INFO("Set publisher and subscriber");

  rosControl.readGoalsFile("checkpoints.txt");
  ROS_INFO("read the file");
  

  ros::Rate loopRate(10.0);

  while(ros::ok())
  {
    ros::spinOnce();
    rosControl.popListToGoal();
    ROS_INFO("Poplist");
    rosControl.publishGoal();
    ROS_INFO("published goal");

    rosControl.waitResultNavigation();
    ROS_INFO("We got a result!");
    
    if(rosControl.getNavigationState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      continue;
    else
      ROS_INFO("Planner failed!!! Using next setpoint.");
    ros::spinOnce();
    loopRate.sleep();
  }
  ros::spin();

  return 0;
}
