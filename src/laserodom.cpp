#include "laserodom/laserodom.h"

#define ODOM_POSE_COVARIANCE                                                   \
  {                                                                            \
    1e-3, 0, 0, 0, 0, 0, 0, 1e-3, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0,     \
        1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e3                      \
  }
#define ODOM_POSE_COVARIANCE2                                                  \
  {                                                                            \
    1e-9, 0, 0, 0, 0, 0, 0, 1e-3, 1e-9, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0,  \
        1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e-9                     \
  }
#define ODOM_TWIST_COVARIANCE                                                  \
  {                                                                            \
    1e-3, 0, 0, 0, 0, 0, 0, 1e-3, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0,     \
        1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e3                      \
  }
#define ODOM_TWIST_COVARIANCE2                                                 \
  {                                                                            \
    1e-9, 0, 0, 0, 0, 0, 0, 1e-3, 1e-9, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0,  \
        1e6, 0, 0, 0, 0, 0, 0, 1e6, 0, 0, 0, 0, 0, 0, 1e-9                     \
  }
Eigen::Vector3d now_pos,last_pos;
Eigen::Vector3d  cal_delta_distence(Eigen::Vector3d odom_pose);

laserodom::laserodom()
{
    ros::NodeHandle private_nh_("~");

    m_prevLDP = NULL;
    SetPIICPParams();

    scan_pos_cal.setZero();
    odom_pos_cal.setZero();
    //odom_increments.clear();

    if(!private_nh_.getParam("odom_frame", odom_frame_))
        odom_frame_ = "odom";
    if(!private_nh_.getParam("base_frame", base_frame_))
        base_frame_ = "base_link";
    if(!private_nh_.getParam("scan_topic", scan_topic_))
        scan_topic_ = "scan";
    if(!private_nh_.getParam("odom_topic", odom_topic_))
        odom_topic_ = "odom_out";
    if(!private_nh_.getParam("odom_topic_sub_", odom_topic_sub_))
        odom_topic_ = "odom_in";
    if (!private_nh_.getParam ("publish_tf", publish_tf_))
    	publish_tf_ = false;

    if (!private_nh_.getParam ("min_laser_dis", min_laser_dis_))
        min_laser_dis_ = 0.1;
    if (!private_nh_.getParam ("max_laser_dis", max_laser_dis_))
        max_laser_dis_ = 25;

    if (!private_nh_.getParam ("det_x_", det_x_))
        det_x_ = 0.04;
    if (!private_nh_.getParam ("det_y_", det_y_))
        det_y_ = 0.04;
    if (!private_nh_.getParam ("det_th_", det_th_))
        det_th_ = 0.03;

    //进行里程计和激光雷达数据的同步
    scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, scan_topic_, 10);
    scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 10);
    scan_filter_->registerCallback(boost::bind(&laserodom::ScanCallBack, this, _1));
    odom_pub = node_.advertise<nav_msgs::Odometry>(odom_topic_, 10);

    odom_sub = private_nh_.subscribe("odom_topic_sub_",1000,&laserodom::odomCallback,this);
    ROS_INFO("start");

}
/*
 * 得到时刻t机器人在里程计坐标下的坐标
*/
bool laserodom::getOdomPose(Eigen::Vector3d& pose, const ros::Time& t)
{
    // Get the robot's pose
    tf::Stamped<tf::Pose> ident (tf::Transform(tf::createQuaternionFromRPY(0,0,0),
                                               tf::Vector3(0,0,0)), t, base_frame_);
    tf::Stamped<tf::Transform> odom_pose;
    try
    {
        tf_.transformPose(odom_frame_, ident, odom_pose);
    }
    catch(tf::TransformException e)
    {
        ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
        return false;
    }
    double yaw = tf::getYaw(odom_pose.getRotation());
    pose << odom_pose.getOrigin().x(),
            odom_pose.getOrigin().y(),
            yaw;
    //pub_msg(pose,path_odom,odom_path_pub_);
    return true;
}

void laserodom::odomCallback(const nav_msgs::Odometry& msg)
{    
    odom_x = msg.pose.pose.position.x;
    odom_y = msg.pose.pose.position.x;

    double roll_odom, pitch_odom, yaw_odom;
    tf::Quaternion quat_odom;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, quat_odom);
    tf::Matrix3x3(quat_odom).getRPY(roll_odom, pitch_odom, yaw_odom);
    odom_th = yaw_odom;
}

//激光数据回调函数
void laserodom::ScanCallBack(const sensor_msgs::LaserScan::ConstPtr &_laserScanMsg)
{
    ROS_INFO("scan_in");

    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();
    static long int dataCnt = 0;
    sensor_msgs::LaserScan scan;
    Eigen::Vector3d odom_pose;              //激光对应的里程计位姿
    Eigen::Vector3d d_point_odom;           //里程计计算的dpose
    Eigen::Vector3d d_point_scan;           //激光的scanmatch计算的dpose
    Eigen::MatrixXd transform_matrix(3,3);  //临时的变量

    double c,s;
    scan = *_laserScanMsg;
/********/
  tf::StampedTransform base_to_laser_;
  base_to_laser_.setIdentity();
  try
  {
    tf_.waitForTransform(base_frame_, scan.header.frame_id, ros::Time(0),ros::Duration(10.0));
    
    tf_.lookupTransform(base_frame_, scan.header.frame_id, ros::Time(0), base_to_laser_);

  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();

  }
/**********/
    //得到对应的里程计数据
    if(!getOdomPose(odom_pose, _laserScanMsg->header.stamp))
        return ;
    ROS_INFO("odom_in");

    //前后两帧里程计的位姿差
    d_point_odom = cal_delta_distence(odom_pose);

    //如果运动的距离太短，则不进行处理．
   /* if(d_point_odom(0) < 0.05 &&
       d_point_odom(1) < 0.05 &&
       d_point_odom(2) < tfRadians(5.0))
    {
        ROS_INFO("odom_out");
        return ;
    }*/
    

    //记录下里程计的增量数据
    //odom_increments.push_back(d_point_odom);


    //把当前的激光数据转换为 pl-icp能识别的数据 & 进行矫正
    //d_point_scan就是用激光计算得到的两帧数据之间的旋转 & 平移
    LDP currentLDP;
    if(m_prevLDP != NULL)
    {
        LaserScanToLDP(&scan,currentLDP);
        d_point_scan = PIICPBetweenTwoFrames(currentLDP,d_point_odom);
	if ((d_point_scan(0)*d_point_scan(0)+d_point_scan(1)*d_point_scan(1)<det_x_*det_y_ && d_point_scan(2) <det_th_))
	{
	ROS_INFO("d_point_scan%lf,%lf,%lf",d_point_scan(0),d_point_scan(1),d_point_scan(2));
		}
	else {
		d_point_scan= d_point_odom;
	}	
    }
    else
    {
        LaserScanToLDP(&scan,m_prevLDP);
    }

    // 构造旋转矩阵 生成三种位姿
    // 两针scan计算本身累计的位姿 for laser_path visualization
    c = cos(scan_pos_cal(2));
    s = sin(scan_pos_cal(2));
    transform_matrix<<c,-s,0,
                      s, c,0,
                      0, 0,1;

	
   //transform form scan_frame to odom_frame
   tf::Transform d_scan_pose;
   //corr_ch.setIdentity();
   createTfFromXYTheta(d_point_scan(0), d_point_scan(1),d_point_scan(2), d_scan_pose);
   d_scan_pose = base_to_laser_*d_scan_pose*base_to_laser_.inverse();
   //ROS_INFO("S%lf,%lf,%lf",scan_pos_cal(0),scan_pos_cal(1),scan_pos_cal(2));

   double yaw = tf::getYaw(d_scan_pose.getRotation());
   d_point_scan << d_scan_pose.getOrigin().x(),
                   d_scan_pose.getOrigin().y(),
                   yaw;
   scan_pos_cal+=(transform_matrix*d_point_scan);
 


  if (publish_tf_)
  {
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(scan_pos_cal(2));

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = odom_frame_;//before map odom
    odom_trans.child_frame_id = base_frame_;//before odom base_link

    odom_trans.transform.translation.x = scan_pos_cal(0);
    odom_trans.transform.translation.y = scan_pos_cal(1);
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_trans);
  }

	
    last_time = current_time;
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = scan_pos_cal(0);
    odom.pose.pose.position.y = scan_pos_cal(1);
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(scan_pos_cal(2));
    odom.twist.twist.linear.x = d_point_scan[0]/dt;
    odom.twist.twist.linear.y = d_point_scan[1]/dt;
    odom.twist.twist.angular.z = d_point_scan[2]/dt;
    if (vx == 0.0 && vy == 0.0 && vth == 0.0) {
      odom.pose.covariance = ODOM_POSE_COVARIANCE2;
      odom.twist.covariance = ODOM_TWIST_COVARIANCE2;
    } else {
      odom.pose.covariance = ODOM_POSE_COVARIANCE;
      odom.twist.covariance = ODOM_TWIST_COVARIANCE;
    }
    odom_pub.publish(odom);

    // 里程计累计的位姿          for odom_path visualization
    //c = cos(odom_pos_cal(2));
    //s = sin(odom_pos_cal(2));
   // transform_matrix<<c,-s,0,
    //                  s, c,0,
    //                  0, 0,1;
    //odom_pos_cal+=(transform_matrix*d_point_odom);

    //ROS_INFO("O%lf,%lf,%lf",odom_pos_cal(0),odom_pos_cal(1),odom_pos_cal(2));

}
void laserodom::createTfFromXYTheta(double x, double y, double theta, tf::Transform& t)
{
  t.setOrigin(tf::Vector3(x, y, 0.0));
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  t.setRotation(q);
}

void laserodom::SetPIICPParams()
{
    //设置激光的范围
    m_PIICPParams.min_reading = min_laser_dis_;
    m_PIICPParams.max_reading = max_laser_dis_;

    //设置位姿最大的变化范围
    m_PIICPParams.max_angular_correction_deg = 20.0;
    m_PIICPParams.max_linear_correction = 1;

    //设置迭代停止的条件
    m_PIICPParams.max_iterations = 50;
    m_PIICPParams.epsilon_xy = 0.000001;
    m_PIICPParams.epsilon_theta = 0.0000001;

    //设置correspondence相关参数
    m_PIICPParams.max_correspondence_dist = 1;
    m_PIICPParams.sigma = 0.01;
    m_PIICPParams.use_corr_tricks = 1;

    //设置restart过程，因为不需要restart所以可以不管
    m_PIICPParams.restart = 0;
    m_PIICPParams.restart_threshold_mean_error = 0.01;
    m_PIICPParams.restart_dt = 1.0;
    m_PIICPParams.restart_dtheta = 0.1;

    //设置聚类参数
    m_PIICPParams.clustering_threshold = 0.2;

    //用最近的10个点来估计方向
    m_PIICPParams.orientation_neighbourhood = 10;

    //设置使用PI-ICP
    m_PIICPParams.use_point_to_line_distance = 1;

    //不进行alpha_test
    m_PIICPParams.do_alpha_test = 0;
    m_PIICPParams.do_alpha_test_thresholdDeg = 5;

    //设置trimmed参数 用来进行outlier remove
    m_PIICPParams.outliers_maxPerc = 0.9;
    m_PIICPParams.outliers_adaptive_order = 0.7;
    m_PIICPParams.outliers_adaptive_mult = 2.0;

    //进行visibility_test 和 remove double
    m_PIICPParams.do_visibility_test = 1;
    m_PIICPParams.outliers_remove_doubles = 1;
    m_PIICPParams.do_compute_covariance = 0;
    m_PIICPParams.debug_verify_tricks = 0;
    m_PIICPParams.use_ml_weights = 0;
    m_PIICPParams.use_sigma_weights = 0;
}



//把激光雷达数据 转换为PI-ICP需要的数据
void laserodom::LaserScanToLDP(sensor_msgs::LaserScan *pScan,
                           LDP& ldp)
{
    int nPts = pScan->intensities.size();
    ldp = ld_alloc_new(nPts);

    for(int i = 0;i < nPts;i++)
    {
        double dist = pScan->ranges[i];
        if(dist > min_laser_dis_ && dist < max_laser_dis_)
        {
            ldp->valid[i] = 1;
            ldp->readings[i] = dist;
        }
        else
        {
            ldp->valid[i] = 0;
            ldp->readings[i] = -1;
        }
        ldp->theta[i] = pScan->angle_min+pScan->angle_increment*i;
    }
    ldp->min_theta = ldp->theta[0];
    ldp->max_theta = ldp->theta[nPts-1];
    ldp->odometry[0] = 0.0;
    ldp->odometry[1] = 0.0;
    ldp->odometry[2] = 0.0;
    ldp->true_pose[0] = 0.0;
    ldp->true_pose[1] = 0.0;
    ldp->true_pose[2] = 0.0;
}


//求两帧之间的icp位姿匹配
Eigen::Vector3d  laserodom::PIICPBetweenTwoFrames(LDP& currentLDPScan,
                                              Eigen::Vector3d tmprPose)
{
    m_prevLDP->odometry[0] = 0.0;
    m_prevLDP->odometry[1] = 0.0;
    m_prevLDP->odometry[2] = 0.0;

    m_prevLDP->estimate[0] = 0.0;
    m_prevLDP->estimate[1] = 0.0;
    m_prevLDP->estimate[2] = 0.0;

    m_prevLDP->true_pose[0] = 0.0;
    m_prevLDP->true_pose[1] = 0.0;
    m_prevLDP->true_pose[2] = 0.0;

    //设置匹配的参数值
    m_PIICPParams.laser_ref = m_prevLDP;
    m_PIICPParams.laser_sens = currentLDPScan;

    m_PIICPParams.first_guess[0] = tmprPose(0);
    m_PIICPParams.first_guess[1] = tmprPose(1);
    m_PIICPParams.first_guess[2] = tmprPose(2);

    m_OutputResult.cov_x_m = 0;
    m_OutputResult.dx_dy1_m = 0;
    m_OutputResult.dx_dy2_m = 0;

    sm_icp(&m_PIICPParams,&m_OutputResult);

    //nowPose在lastPose中的坐标
    Eigen::Vector3d  rPose;
    if(m_OutputResult.valid)
    {
        //得到两帧激光之间的相对位姿
        rPose(0)=(m_OutputResult.x[0]);
        rPose(1)=(m_OutputResult.x[1]);
        rPose(2)=(m_OutputResult.x[2]);
//        std::cout <<"Iter:"<<m_OutputResult.iterations<<std::endl;
//        std::cout <<"Corr:"<<m_OutputResult.nvalid<<std::endl;
//        std::cout <<"Erro:"<<m_OutputResult.error<<std::endl;
//        std::cout <<"PI ICP GOOD"<<std::endl;
    }
    else
    {
        std::cout <<"PI ICP Failed! use d_odom "<<std::endl;
        rPose = tmprPose;
    }
    //更新
    //ld_free(m_prevLDP);
    m_prevLDP = currentLDPScan;
    return rPose;
}

Eigen::Vector3d  cal_delta_distence(Eigen::Vector3d odom_pose)
{

    Eigen::Vector3d d_pos;  //return value
    now_pos = odom_pose;
    Eigen::Matrix3d Tnow,Tprev;
    double theta = last_pos(2);
    double x = last_pos(0);
    double y = last_pos(1);
    //前一次的位姿
    Tprev << cos(theta),-sin(theta),x,
             sin(theta), cos(theta),y,
              0,          0,       1;
    //当前的位姿
    x = now_pos(0);
    y = now_pos(1);
    theta = now_pos(2);
    Tnow << cos(theta),-sin(theta),x,
            sin(theta), cos(theta),y,
            0,          0,       1;
    //相对位姿
    Eigen::Matrix3d T = Tprev.inverse() * Tnow;
    d_pos(0) = T(0,2);
    d_pos(1) = T(1,2);
    d_pos(2) = atan2(T(1,0),T(0,0));
    last_pos = now_pos;
    return d_pos;
}

int main(int argc,char** argv)
{

    ros::init(argc, argv, "lo");
    ros::Time::init();
    ros::NodeHandle n;
    laserodom lo;
    ros::spin();
    return 0;
}
