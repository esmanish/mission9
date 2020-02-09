/**
 * @file gps_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <thread>
#include <chrono>
#include <mutex>

using namespace std;
using namespace std::chrono_literals;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

sensor_msgs::NavSatFix current_global;
void global_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    current_global = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 10, global_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher global_pos_pub = nh.advertise<geographic_msgs::GeoPoseStamped>("mavros/setpoint_position/global", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    // First GPS position reported by mavros
    sensor_msgs::NavSatFix start_global;

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    geographic_msgs::GeoPoseStamped global_target;

    //send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                    last_request = ros::Time::now();
                    start_global = current_global;
                    cout << current_global.latitude << " "
                         << current_global.longitude << " "
                         << current_global.altitude << endl;

                    global_target.pose.position.latitude = current_global.latitude;
                    global_target.pose.position.longitude = current_global.longitude;
                    global_target.pose.position.altitude = 2; //current_global.altitude;

                    break;
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    //mutex mtx;

    thread t{[&global_pos_pub, &global_target/*, &mtx*/] {
        while (ros::ok())
        {
            //std::unique_lock<mutex> lk{mtx};
            global_pos_pub.publish(global_target);
            std::this_thread::sleep_for(100ms);

            //ros::spinOnce();
        }
    }};

    while (ros::ok())
    {
        double latitude, longitude, altitude;
        // std::unique_lock<mutex> lk{mtx};

        // cin >> latitude >> longitude >> altitude;

        // global_target.pose.position.latitude = latitude;
        // global_target.pose.position.longitude = longitude;
        // global_target.pose.position.altitude = altitude;

        // std::string input;
        // std::cout << "Test: ";
        // std::getline(std::cin, input);

        ros::spinOnce();
        rate.sleep();
    }

    t.join();
    return 0;
}
