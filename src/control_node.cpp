#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "diffdrive/GetGoal.h"
#include <math.h>

#define RUN_PERIOD_DEFAULT 0.1
#define NAME_OF_THIS_NODE "control_node"

//-----------------------------------------------------------------
//-----------------------------------------------------------------

class ROSnode {
private: 
    ros::NodeHandle Handle;
    ros::Subscriber poseSub;
    ros::Publisher cmdPub;
    ros::ServiceClient goalCl;
    bool position, orientation, gotPose;
    geometry_msgs::Pose goal;
    
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
public:    
    void Prepare();
    void RunContinuously();
    void Shutdown();
};

//-----------------------------------------------------------------
//-----------------------------------------------------------------

void ROSnode::Prepare() {
    position = false;
    orientation = false;
    gotPose = false;
    poseSub = Handle.subscribe("/pose", 10, &ROSnode::poseCallback, this);    
    cmdPub = Handle.advertise<geometry_msgs::Twist>("cmd_auto", 10);
    goalCl = Handle.serviceClient<diffdrive::GetGoal>("goal");
    
    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}


void ROSnode::RunContinuously() {
  ROS_INFO("Node %s running continuously.", ros::this_node::getName().c_str());
   
  ros::spin();
}

void ROSnode::Shutdown() {
  ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void ROSnode::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    geometry_msgs::Twist out;
    out.angular.x = 0.0;
    out.angular.y = 0.0;
    out.angular.z = 0.0;
    out.linear.x = 0.0;
    out.linear.y = 0.0;
    out.linear.z = 0.0;
    
    if(!gotPose) {
        diffdrive::GetGoal srv;
        srv.request.go = true;
        if(goalCl.call(srv)) {
            goal = srv.response.goal;
            gotPose = true;
            position = false;
            orientation = false;
        }
        else {
            ROS_INFO("Destination reached");
            return;
        }
    }
    if(!orientation) {
        double th = atan2(goal.position.y - msg->pose.position.y, goal.position.x - msg->pose.position.x);
        double q0 = msg->pose.orientation.w;
//         double q1 = msg->pose.orientation.x; /* 0.0 */
//         double q2 = msg->pose.orientation.y; /* 0.0 */
        double q3 = msg->pose.orientation.z;
//         double roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
//         double pitch = asin(2*(q0*q2 - q3*q1));
//         double yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
        double yaw = atan2(2*(q0*q3), 1 - 2*(q3*q3));
        if(fabs(yaw - th) < 0.005)
            orientation = true;
        else
            out.angular.z = 0.03;
    }
    if(!position && orientation) {
        double d2 = pow(goal.position.x - msg->pose.position.x, 2) + pow(goal.position.y - msg->pose.position.y, 2);
        if(d2 >= 1.8*1.8)
            out.linear.x = 0.15;
        if(d2 < 1.8*1.8 && d2 >= 1.4*1.4)
            out.linear.x = 0.12;
        if(d2 < 1.4*1.4 && d2 >= 1.0*1.0)
            out.linear.x = 0.09;
        if(d2 < 1.0*1.0 && d2 >= 0.6*0.6)
            out.linear.x = 0.06;
        if(d2 < 0.6*0.6 && d2 >= 0.2*0.2)
            out.linear.x = 0.03;
        if(d2 < 0.2*0.2) {
            out.linear.x = 0.0;
            position = true;
        }
    }
    if(position && orientation)
        gotPose = false;
    
    cmdPub.publish(out);
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
