#define RUN_PERIOD_DEFAULT 1.0
#define NAME_OF_THIS_NODE "send_plan"

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

class ROSnode {
private:
    ros::NodeHandle Handle;
    nav_msgs::Path mPath;
    ros::Publisher pathPub;
public:
    void Prepare();
    void DoIt();
};

//-----------------------------------------------------------------
//-----------------------------------------------------------------

void ROSnode::Prepare() {
    pathPub = Handle.advertise<nav_msgs::Path>("/plan", 10);
    
    geometry_msgs::PoseStamped element;
    
    element.pose.position.x = 5.0;
    element.pose.position.y = 2.0;
    mPath.poses.push_back(element);
    
    element.pose.position.x = 2.0;
    element.pose.position.y = 5.0;
    mPath.poses.push_back(element);
    
    element.pose.position.x = -5.0;
    element.pose.position.y = 2.0;
    mPath.poses.push_back(element);
    
    element.pose.position.x = 0.0;
    element.pose.position.y = 0.0;
    mPath.poses.push_back(element);
}

void ROSnode::DoIt() { pathPub.publish(mPath); }

int main(int argc, char **argv) {
    ros::init(argc, argv, NAME_OF_THIS_NODE);
    
    ROSnode mNode;
    
    mNode.Prepare();
    ros::Rate LoopRate(1.0);
    while (ros::ok()) {
        mNode.DoIt();
        ros::spinOnce();
        LoopRate.sleep();
    }
    return (0);
}