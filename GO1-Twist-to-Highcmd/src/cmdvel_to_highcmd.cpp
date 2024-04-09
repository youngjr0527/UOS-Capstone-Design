#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub;
unitree_legged_msgs::HighCmd high_cmd_ros;


void cmdCallback(const geometry_msgs::Twist& msg){
    double speedX = msg.linear.x;
    double speedY = msg.linear.y;
    double speedYaw = msg.angular.z;


    high_cmd_ros.head[0] = 0xFE;
    high_cmd_ros.head[1] = 0xEF;
    // high_cmd_ros.levelFlag = HIGHLEVEL;
    high_cmd_ros.levelFlag = 0x00;
    high_cmd_ros.mode = 0;
    high_cmd_ros.gaitType = 0;
    high_cmd_ros.speedLevel = 0;
    high_cmd_ros.footRaiseHeight = 0;
    high_cmd_ros.bodyHeight = 0;
    high_cmd_ros.euler[0] = 0;
    high_cmd_ros.euler[1] = 0;
    high_cmd_ros.euler[2] = 0;
    high_cmd_ros.velocity[0] = 0.0f;
    high_cmd_ros.velocity[1] = 0.0f;
    high_cmd_ros.yawSpeed = 0.0f;
    high_cmd_ros.reserve = 0;

    
    // walk
    high_cmd_ros.mode = 2;
    high_cmd_ros.gaitType = 1;
    high_cmd_ros.velocity[0] = speedX;
    high_cmd_ros.velocity[1] = speedY;
    high_cmd_ros.yawSpeed = speedYaw;
    high_cmd_ros.footRaiseHeight = 0.1;
    
    

    pub.publish(high_cmd_ros);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmdvel_to_highcmd");
    ros::NodeHandle nh;


    pub = nh.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 1);
    ros::Subscriber sub = nh.subscribe("cmd_vel", 1, cmdCallback);

    ros::spin();
    return 0;
}