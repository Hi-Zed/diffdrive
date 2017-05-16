#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "diffdrive/GetGoal.h"
#include "tf/transform_listener.h"
#include <math.h>

#define RUN_PERIOD_DEFAULT 0.05
#define NAME_OF_THIS_NODE "control_node_tf"

//-----------------------------------------------------------------
//-----------------------------------------------------------------

class ROSnode {
private: 
    ros::NodeHandle Handle;
    ros::Publisher cmdPub;
    ros::ServiceClient goalCl;
    tf::TransformListener tf;
    bool position, orientation, gotPose;
    geometry_msgs::Pose goal;
    
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void control();
public:    
    void Prepare();
    void RunPeriodically();
    void Shutdown();
};

//-----------------------------------------------------------------
//-----------------------------------------------------------------

void ROSnode::Prepare() {
    position = false;
    orientation = false;
    gotPose = false;
    cmdPub = Handle.advertise<geometry_msgs::Twist>("/willy2/cmd_vel", 10);
    goalCl = Handle.serviceClient<diffdrive::GetGoal>("goal");
    
    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}


void ROSnode::RunPeriodically() {
    //Frequency of execution of the node (i.e. 1 Hz)
    ros::Rate LoopRate(1.0/RUN_PERIOD_DEFAULT);
    
    ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), RUN_PERIOD_DEFAULT, 1.0/RUN_PERIOD_DEFAULT);
    
    //Main execution loop
    while (ros::ok()) {
        control();
        //Single internal loop and sleep to maintain the frequency
        ros::spinOnce();
        LoopRate.sleep();
    }
}

void ROSnode::Shutdown() {
  ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void ROSnode::control() {
    tf::StampedTransform transform;
    try{ tf.lookupTransform("/world", "/base_link", ros::Time(0), transform); }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        return;
    }
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
        double th = atan2(goal.position.y - transform.getOrigin().y(), goal.position.x - transform.getOrigin().x());
        double yaw = tf::getYaw(transform.getRotation());
        if(fabs(yaw - th) < 0.005)
            orientation = true;
        else
            out.angular.z = 0.03;
    }
    if(!position && orientation) {
        double d2 = pow(goal.position.x - transform.getOrigin().x(), 2) + pow(goal.position.y - transform.getOrigin().y(), 2);
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
  mNode.RunPeriodically();
  mNode.Shutdown();
  
  return (0);
}