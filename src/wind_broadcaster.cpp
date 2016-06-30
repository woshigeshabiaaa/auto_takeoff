//
// Created by flyinsky on 16-6-24.
//

#include <ros/ros.h>
#include <auto_takeoff/WindGust.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "wind_publisher");
    ros::NodeHandle nh;

    srand(0);

    ros::Publisher wind_gust_pub = nh.advertise<auto_takeoff::WindGust>("/wind", 1000);

    ros::Rate rate(10);

    while (ros::ok()) {
        auto_takeoff::WindGust wind_gust;

        wind_gust.wind_gust_mean.data = 5 * float(rand()) / RAND_MAX;

        wind_gust.wind_gust_direction.x = double(rand()) / RAND_MAX - 0.5;
        wind_gust.wind_gust_direction.y = double(rand()) / RAND_MAX - 0.5;
        wind_gust.wind_gust_direction.z = double(rand()) / RAND_MAX - 0.5;

        wind_gust.wind_gust_on.data = true;

        wind_gust_pub.publish(wind_gust);

        ROS_INFO_STREAM("Wind is ON, the mean is: " << wind_gust.wind_gust_mean.data
                        << " ; the direction is : ["
                        << wind_gust.wind_gust_direction.x << " , "
                        << wind_gust.wind_gust_direction.y << " , "
                        << wind_gust.wind_gust_direction.z << " ] ");
        ros::spinOnce();
        rate.sleep();

    }


}