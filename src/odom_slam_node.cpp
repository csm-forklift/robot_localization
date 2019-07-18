#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>



int main(int argc, char** argv){
    ROS_INFO("Initializing odom_slam_node");
    ros::init(argc,argv,"odom_slam_node");

    ros::NodeHandle nh_;
    ros::Publisher odom_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/odom_slam",1);
    //ros::Publisher odom_pub = nh_.advertise<nav_msgs::Odometry>("/odom_slam",1);
    nav_msgs::Odometry temp_odom;
    geometry_msgs::PoseWithCovarianceStamped temp_pose;
    temp_pose.header.frame_id ="odom";
    temp_pose.header.seq = 0;
    //temp_odom.header.frame_id = "/odom";
    //temp_odom.child_frame_id = "/base_link2";
    //temp_odom.header.seq =0;

    tf::TransformListener listener;
    ros::Rate rate(50.0);
    while(nh_.ok()){
        tf::StampedTransform transform;
        try {
            ros::Time now = ros::Time(0);
            listener.waitForTransform("/odom","/base_link",now,ros::Duration(0.5));
            listener.lookupTransform("/odom","/base_link",now,transform);
        }
        catch(tf::TransformException &ex) {
            ROS_INFO("BROKEN!!");
             ROS_ERROR("%s",ex.what());
             ros::Duration(1.0).sleep();
             continue;
        }
        temp_pose.header.seq++;
        temp_pose.header.stamp = ros::Time::now();
        temp_pose.pose.pose.position.x = transform.getOrigin().x();
        temp_pose.pose.pose.position.y = transform.getOrigin().y();
        temp_pose.pose.pose.position.z = transform.getOrigin().z();
        temp_pose.pose.pose.orientation.w = transform.getRotation().w();
        temp_pose.pose.pose.orientation.x = transform.getRotation().x();
        temp_pose.pose.pose.orientation.y = transform.getRotation().y();
        temp_pose.pose.pose.orientation.z = transform.getRotation().z();
        temp_pose.pose.covariance[0] = 0.0001;
        odom_pub.publish(temp_pose);
        
        /*temp_odom.header.seq++;
        temp_odom.header.stamp = ros::Time(0);
        temp_odom.pose.pose.position.x = transform.getOrigin().x();
        temp_odom.pose.pose.position.y = transform.getOrigin().y();
        temp_odom.pose.pose.position.z = transform.getOrigin().z();
        temp_odom.pose.pose.orientation.w = transform.getRotation().w();
        temp_odom.pose.pose.orientation.x =transform.getRotation().x();
        temp_odom.pose.pose.orientation.y =transform.getRotation().y();
        temp_odom.pose.pose.orientation.z =transform.getRotation().z();
        odom_pub.publish(temp_odom);        
*/
        rate.sleep();
    }


    return 0;
}
