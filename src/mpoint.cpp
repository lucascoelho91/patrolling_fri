#include <iostream>
#include <list>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>


class Controller
{
  private: 

    std::list< std::vector<double> > listGoals;

    ros::Publisher movebasePub;  
    ros::Subscriber movebaseStatus;

    ros::NodeHandle *node;
    
    geometry_msgs::PoseStamped currentGoal;

  public:

    void moveBaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
    {
      if(msg->status_list[0].status == 3)
      {
        if (!this->popListToGoal())
        {
          printf("We reached the end of the list of goals! Nothing to do anymore.\n");
          exit(0);
        }
        else
        {
          this->publishGoal();
        }
      }
    }

    void publishGoal()
    {
      movebasePub.publish(currentGoal);
    }

    void setCurrentGoal(double x, double y, double theta=0)
    {
      currentGoal.pose.position.x = x;
      currentGoal.pose.position.y = y;
      currentGoal.pose.orientation.z = theta;
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

    void setNodeHandle(ros::NodeHandle *n)
    {
      node = n;
    }
    Controller(ros::NodeHandle *n)
    {
      node = n;
    }
};


int main(int argc, char** argv)
{

  ros::init(argc, argv, "navPoints");
  ros::NodeHandle node;

  geometry_msgs::PoseStamped posegoal;

  Controller rosControl(&node);

  rosControl.setMoveBaseStatusSubscriber("move_base/status", 10);
  rosControl.setMoveBasePublisher("/move_base_simple/goal", 10);

  rosControl.readGoalsFile("checkpoints.txt");
  rosControl.popListToGoal();
  rosControl.publishGoal();

  ros::spin();

  return 0;
  
}