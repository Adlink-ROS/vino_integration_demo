#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "object_msgs/ObjectsInBoxes.h"

ros::Publisher cmd_vel_pub;

enum ACTION
{
    CMD_ROTATE_RIGHT,
    CMD_ROTATE_LEFT,
};

std::map<std::string, int> obj2act =
{
    { "person", CMD_ROTATE_RIGHT },
    { "chair",  CMD_ROTATE_LEFT }
};

void pub_cmdvel(int idx)
{
    geometry_msgs::Twist cmd_vel;
    switch(idx) {
       case CMD_ROTATE_RIGHT:
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.linear.z = 0;
            cmd_vel.angular.x = 0;
            cmd_vel.angular.y = 0;
            cmd_vel.angular.z = -0.3;
            std::cout << " Turn right !." << std::endl;
           break;
        case CMD_ROTATE_LEFT:
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.linear.z = 0;
            cmd_vel.angular.x = 0;
            cmd_vel.angular.y = 0;
            cmd_vel.angular.z = 0.3;
            std::cout << " Turn left !." << std::endl;
            break;
        default:
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.linear.z = 0;
            cmd_vel.angular.x = 0;
            cmd_vel.angular.y = 0;
            cmd_vel.angular.z = 0;
            std::cout << " Stop !." << std::endl;
            break;

    }
    cmd_vel_pub.publish(cmd_vel);
}

void obj_2_cmdvel(std::string obj_name)
{
    auto relation = obj2act.find(obj_name);
    if(relation != obj2act.end())
    {
        pub_cmdvel(obj2act[obj_name]);
    }
    else
    {
        pub_cmdvel(-1);
    }
}

void objDetectCallback(const object_msgs::ObjectsInBoxes::ConstPtr msg)
{
    int cnt = 0;
    for(auto & obj : msg->objects_vector)
    {
        std::cout << "["<< ++cnt <<"] I see a '" << obj.object.object_name << "'." << std::endl;
        obj_2_cmdvel(obj.object.object_name);
    }
}

int main(int argc, char **argv) {
    printf("subscribe\n");
    ros::init(argc, argv, "integrator");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/ros_openvino_toolkit/detected_objects", 1000, objDetectCallback);
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
    ros::spin();
    return 0;
}