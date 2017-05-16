#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "diffdrive/GetGoal.h"

#define RUN_PERIOD_DEFAULT 0.1
#define NAME_OF_THIS_NODE "sequencer_node"

//-----------------------------------------------------------------
//-----------------------------------------------------------------

class ROSnode {
private:
    ros::NodeHandle Handle;
    ros::Subscriber planSub;
    ros::ServiceServer goalSr;
    int lastpose;
    bool gotPlan;
    nav_msgs::Path mPath;
    
    void planCallback(const nav_msgs::Path::ConstPtr& msg);
    bool goalService(diffdrive::GetGoal::Request &req, diffdrive::GetGoal::Response &res);
public:
    void Prepare();
    void RunContinuously();
    void Shutdown();
};

//-----------------------------------------------------------------
//-----------------------------------------------------------------

void ROSnode::Prepare() {
    lastpose = 0;
    gotPlan = false;
    planSub = Handle.subscribe("/plan", 10, &ROSnode::planCallback, this);
    goalSr = Handle.advertiseService("goal", &ROSnode::goalService, this);
    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}


void ROSnode::RunContinuously() {
    ROS_INFO("Node %s running continuously.", ros::this_node::getName().c_str());
    ros::spin();
}

void ROSnode::Shutdown() {
    ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void ROSnode::planCallback(const nav_msgs::Path::ConstPtr& msg) {
    if(!gotPlan) {
        mPath.poses = msg->poses;
        gotPlan = true;
        ROS_INFO("Plan received");
    }
}

bool ROSnode::goalService(diffdrive::GetGoal::Request &req, diffdrive::GetGoal::Response &res) {
    if(lastpose >= mPath.poses.size() || !gotPlan)
        return false;
    if(req.go) {
        res.goal = mPath.poses[lastpose].pose;
        lastpose++;
    }
    return true;
}

//-----------------------------------------------------------------
//-----------------------------------------------------------------

int main(int argc, char **argv) {
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  ROSnode mNode;
   
  mNode.Prepare();
  mNode.RunContinuously();
  mNode.Shutdown();
  
  return (0);
}