#include <cstdlib>

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <aruco/aruco.h>
#include <iostream>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>

#define FACTOR  0.6
#define MINRC   1100
#define BASERC  1500
#define MAXRC   1900

image_transport::Subscriber sub;   // Subscriber to bottom camera
ros::Subscriber mavros_state_sub;  // Subscriber to flight mode
ros::Publisher pub;  // RC publisher
ros::Time lastTime;  // Time control

double Roll, Pitch;

// Flight mode
std::string mode;
bool guided;
bool armed;


void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {

        // Create RC msg
        mavros_msgs::OverrideRCIn msg;

        Roll = BASERC;
        Pitch = BASERC;

        // Calculate Roll and Pitch depending on the mode
        if (mode == "LOITER") {
            Roll = BASERC;
            Pitch = BASERC;
        }

        msg.channels[0] = Roll;     //Roll
        msg.channels[1] = Pitch;    //Pitch
        msg.channels[2] = BASERC;   //Throttle
        msg.channels[3] = 0;        //Yaw
        msg.channels[4] = 0;
        msg.channels[5] = 0;
        msg.channels[6] = 0;
        msg.channels[7] = 0;

        pub.publish(msg);
    }

    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void mavrosStateCb(const mavros_msgs::StateConstPtr &msg) {
    if (msg->mode == std::string("CMODE(0)"))
        return;
    //ROS_INFO("I heard: [%s] [%d] [%d]", msg->mode.c_str(), msg->armed, msg->guided);
    mode = msg->mode;
    guided = msg->guided == 128;
    armed = msg->armed == 128;
}


int main(int argc, char **argv) {

    sleep(10);

    int rate = 10;

    ros::init(argc, argv, "mavros_takeoff");
    ros::NodeHandle nodeHandle;

    ros::Rate r(rate);

    ///////////////ARUCO//////////////////
    lastTime = ros::Time::now();
    image_transport::ImageTransport it(nodeHandle);
    sub = it.subscribe("/erlecopter/bottom/image_raw", 1, imageCallback);
    mavros_state_sub = nodeHandle.subscribe("/mavros/state", 1, mavrosStateCb);
    pub = nodeHandle.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);

    //////////////////////////////////////
    ros::ServiceClient cl = nodeHandle.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode srv_setMode;

    /////////////////GUIDED/////////////////////
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "GUIDED";
    if (cl.call(srv_setMode)) {
        ROS_ERROR("setmode send ok %d value:", srv_setMode.response.success);
    } else {
        ROS_ERROR("Failed SetMode");
        return -1;
    }

    ///////////////////ARM//////////////////////
    ros::ServiceClient arming_cl = nodeHandle.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool srv;
    srv.request.value = true;
    if (arming_cl.call(srv)) {
        ROS_ERROR("ARM send ok %d", srv.response.success);
    } else {
        ROS_ERROR("Failed arming or disarming");
    }

    /////////////////TAKEOFF////////////////////
    ros::ServiceClient takeoff_cl = nodeHandle.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = 10;
    srv_takeoff.request.latitude = 0;
    srv_takeoff.request.longitude = 0;
    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw = 0;
    if (takeoff_cl.call(srv_takeoff)) {
        ROS_ERROR("srv_takeoff send ok %d", srv_takeoff.response.success);
    } else {
        ROS_ERROR("Failed Takeoff");
    }

    /////////////////DO STUFF///////////////////
    sleep(3);

    ///////////////////LOITER/////////////////////
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "LOITER";
    if (cl.call(srv_setMode)) {
        ROS_ERROR("setmode send ok %d value:", srv_setMode.response.success);
    } else {
        ROS_ERROR("Failed SetMode");
        return -1;
    }

/*      ///////////////////LAND/////////////////////
    ros::ServiceClient land_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land;
    srv_land.request.altitude = 10;
    srv_land.request.latitude = 0;
    srv_land.request.longitude = 0;
    srv_land.request.min_pitch = 0;
    srv_land.request.yaw = 0;
    if(land_cl.call(srv_land)){
        ROS_INFO("srv_land send ok %d", srv_land.response.success);
    }else{
        ROS_ERROR("Failed Land");
    }*/

    while (nodeHandle.ok()) {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

