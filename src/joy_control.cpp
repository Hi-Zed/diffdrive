#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"


#define RUN_PERIOD_DEFAULT 0.1
#define NAME_OF_THIS_NODE "joy_control"

class ROSnode {
private:
    double maxLinear, maxAngular;
    ros::NodeHandle Handle;
    ros::Subscriber joySub;
    ros::Publisher cmdPub;
    
    geometry_msgs::Twist out;
            
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
public:
    void Prepare();
    void RunContinuously();
    void RunPeriodically();
};

void ROSnode::Prepare() {
    joySub = Handle.subscribe("joy", 10, &ROSnode::joyCallback, this);    
    cmdPub = Handle.advertise<geometry_msgs::Twist>("cmd_joy", 10);
    
    Handle.param("/max_linear", maxLinear, 1.0);
    Handle.param("/max_angular", maxAngular, 1.0);    
    
    
    ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void ROSnode::joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
//     geometry_msgs::Twist out;
    //left analog up/down
    out.linear.x = maxLinear * msg->axes[1];
    
    //right analog left/right
    out.angular.z = maxAngular * msg->axes[3];
    
//     cmdPub.publish(out);
}

void ROSnode::RunContinuously() {
    ros::spin();
}

void ROSnode::RunPeriodically () {
    ros::Rate r(10);
    ros::Rate r1(1/RUN_PERIOD_DEFAULT);
    
    while(ros::ok()) {
        ros::spinOnce();
        cmdPub.publish(out);
        r.sleep();
    }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  ROSnode mNode;
   
  mNode.Prepare();
  //mNode.RunContinuously();
  mNode.RunPeriodically();
  
  return (0);
}










