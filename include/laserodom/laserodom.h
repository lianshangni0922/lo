#ifndef LASER_ODOM_H
#define LASER_ODOM_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <nav_msgs/Odometry.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <csm/csm_all.h>  // csm defines min and max, but Eigen complains
class laserodom
{
public:
    laserodom();
    LDP m_prevLDP;
    sm_params m_PIICPParams;
    sm_result m_OutputResult;
    Eigen::Vector3d scan_pos_cal;
    Eigen::Vector3d odom_pos_cal;

    //std::vector<Eigen::Vector3d> odom_increments; //save inc_odom
    //TIME

    std::string odom_frame_;
    std::string base_frame_;
    std::string scan_topic_; 
    std::string odom_topic_; 
    std::string odom_topic_sub_; 

    bool publish_tf_;

    ros::NodeHandle node_;   
    ros::Publisher odom_pub ;
    tf::TransformListener tf_;
    tf::TransformBroadcaster odom_broadcaster;
    ros::Subscriber  odom_sub;


    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;


    void ScanCallBack(const sensor_msgs::LaserScan::ConstPtr&laserodom);
    void odomCallback(const nav_msgs::Odometry& msg);
    bool getOdomPose(Eigen::Vector3d& pose, const ros::Time& t);


    void SetPIICPParams();
    void LaserScanToLDP(sensor_msgs::LaserScan *pScan,
                                 LDP& ldp);
    Eigen::Vector3d  PIICPBetweenTwoFrames(LDP& currentLDPScan,   
                                          Eigen::Vector3d tmprPose);

     double vx ;     // x方向的速度
     double vy ;     // y方向的速度
     double vth ;    // 角速度
     double dt ;     // time

     double min_laser_dis_, max_laser_dis_;
     double det_x_,det_y_,det_th_;//
     double odom_x,odom_y,odom_th;   

    ros::Time current_time, last_time;
    void createTfFromXYTheta(double x, double y, double theta, tf::Transform& t);

};




#endif
