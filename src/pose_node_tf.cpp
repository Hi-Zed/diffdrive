#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_broadcaster.h"
#include <math.h>

#define RUN_PERIOD_DEFAULT 0.1
#define NAME_OF_THIS_NODE "pose_node_tf"

class ROSnode {
private: 
    ros::NodeHandle Handle;
    ros::Subscriber odomSub;
    ros::Publisher posePub;
    double x, y, yaw, roll, pitch, t;
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

public:
    bool Prepare();
    void RunContinuously();
    void Shutdown();
};

//-----------------------------------------------------------------
//-----------------------------------------------------------------

bool ROSnode::Prepare() {
    ROS_INFO("%s", ros::this_node::getName().c_str());
    if (!Handle.getParam(ros::this_node::getName()+"/x", x)) return false;
    if (!Handle.getParam(ros::this_node::getName()+"/y", y)) return false;
    if (!Handle.getParam(ros::this_node::getName()+"/yaw", yaw)) return false;
    roll = 0.0;
    pitch = 0.0;
    t = -1.0;
    odomSub = Handle.subscribe("/willy2/odom", 10, &ROSnode::odomCallback, this);
    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
    return true;
}


void ROSnode::RunContinuously() {
  ROS_INFO("Node %s running continuously.", ros::this_node::getName().c_str());
   
  ros::spin();
}

void ROSnode::Shutdown() {
  ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}


void ROSnode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if(t < 0) {
        t = msg->header.stamp.toSec();
        return;
    }
    float v = msg->twist.twist.linear.x;
    float w = msg->twist.twist.angular.z;
    double dt = msg->header.stamp.toSec() - t;
    x = x + v*cos(yaw)*dt;
    y = y + v*sin(yaw)*dt;
    yaw = yaw + w*dt;
    
    static tf::TransformBroadcaster tr;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(x, y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    transform.setRotation(q);
    tr.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "world", "base_link"));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  ROSnode mNode;
   
  if(!mNode.Prepare()) return (-1);
  mNode.RunContinuously();
  mNode.Shutdown();
  
  return (0);
}