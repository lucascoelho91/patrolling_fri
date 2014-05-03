#include <iostream>
#include <list>
#include <vector>
#include <stdio.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <tf/tf.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <sound_play/sound_play.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Controller
{
  private: 

    std::list< std::vector<double> > listGoals;
  
    ros::Publisher velPub;   

    ros::Subscriber movebaseStatus;
    ros::Subscriber blobSub;

    ros::NodeHandle *node;
    sound_play::SoundClient sc;
    
    MoveBaseClient ac;
    move_base_msgs::MoveBaseGoal currentGoal;

  public:

    void publishGoal()
    {
      ac.sendGoal(currentGoal);
      //ROS_INFO("Published goal. x: %f y: %f", currentGoal.target_pose.pose.position.x,
      //                                        currentGoal.target_pose.pose.position.y);
    }

    void blobCallback(const cmvision::Blobs::ConstPtr& msg)
    {
      cmvision::Blob blob;


      if(msg->blob_count==0)
      {
        ROS_INFO("No blobs detected.");
        sc.stopSaying(str1);
        return;
      }

      blob = msg->blobs[0];

      if(blob.area < 500)
      {
        ROS_INFO("Hmm... That object looks suspicious but I think it's just something else. Area = %u\n", blob.area);
        sc.stopSaying(str1);
        return; 
      }

      const char *str1 = "Intruder alert!!!";
      sc.repeat(str1);
    }

    bool waitResultNavigation(ros::Duration timeout)
    {
      return ac.waitForResult(timeout);
    }

    actionlib::SimpleClientGoalState getNavigationState()
    {
      return ac.getState();
    }

    void setCurrentGoal(double x, double y, double theta=0)
    {
      currentGoal.target_pose.header.frame_id = "map";
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

    bool popListToGoalAndPush() //creates a circular queue
    {
      std::vector<double> p2(3);
      if(listGoals.size() <= 0)
        return 0;
      else
      {
        p2 = listGoals.front();
        setCurrentGoal(p2[0], p2[1], p2[2]);
        listGoals.pop_front();
        listGoals.push_back(p2);
        return 1;
      }
    }

    void setMoveBasePublisher(std::string topic, int buffersize)
    {
      movebasePub = node->advertise<geometry_msgs::PoseStamped>(topic, buffersize);
    }

    bool readGoalsFile(std::string fileName)
    {
      FILE * pFile;
      pFile = fopen(fileName.c_str(), "r");
      if(pFile==NULL){
        ROS_INFO("File not found.");
        exit(1);
      }

      std::vector<double> pin(3);
      for (;!feof(pFile);)
      {
        fscanf(pFile, "%lf %lf %lf", &pin[0], &pin[1], &pin[2]);
        listGoals.push_back(pin);
        //ROS_INFO("Goal: x: %f, y: %f, theta: %f", pin[0], pin[1], pin[2]);
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
        ROS_INFO("Error while trying to add to the list.\n");
        return 0;
      }
      return 1;
    }

    void waitForMoveBaseServer()
    {
      ac.waitForServer(ros::Duration(5.0));
    }

    void setBlobSubscriber(std::string topic)
    {
      ros::Subscriber blobSub = node->subscribe(topic, 100, &Controller::blobCallback, this);
    }

    void setCmdVelPublisher(std::string topic)
    {
      ros::Publisher velPub = node->advertise<geometry_msgs::Twist>(topic, 100);
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

  rosControl.setMoveBasePublisher("/move_base_simple/goal", 100);
  ROS_INFO("Set publisher and subscriber");

  rosControl.setBlobSubscriber("/blobs");
  rosControl.setCmdVelPublisher("/cmd_vel");
  ROS_INFO("Set blobs and /cmd_vel topics");

  rosControl.readGoalsFile("checkpoints.txt");
  ROS_INFO("read the file");

  ros::Rate loopRate(10.0);

  while(ros::ok())
  {
    ros::spinOnce();
    rosControl.popListToGoalAndPush();
    rosControl.publishGoal();
    
    if (rosControl.waitResultNavigation(ros::Duration(100.0)))
      ROS_INFO("Goal accomplished.");
    else
      ROS_INFO("Ooops, something wrong happened.");
    
    if(rosControl.getNavigationState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the base moved");
    else
      ROS_INFO("Planner failed!!! Using next setpoint.");
    ros::spinOnce();
    loopRate.sleep();
  }
  ros::spin();

  return 0;
}
