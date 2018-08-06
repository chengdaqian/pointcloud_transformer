#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/Imu.h>

ros::Publisher pc2_msg_pub_;
bool is_filter_stationary_;
bool is_imu_init_ = false;
tf::TransformListener* listener_ptr_ = NULL;

struct imu_data{
    double lin_x;
    double lin_y;
    double lin_z;
    double ang_x;
    double ang_y;
    double ang_z;

    imu_data() : lin_x(0.0), lin_y(0.0),lin_z(0.0),ang_x(0.0), ang_y(0.0), ang_z(0.0){}
    imu_data(const sensor_msgs::Imu& msg){
        lin_x = msg.linear_acceleration.x;
        lin_y = msg.linear_acceleration.y;
        lin_z = msg.linear_acceleration.z;
        ang_x = msg.angular_velocity.x;
        ang_y = msg.angular_velocity.y;
        ang_z = msg.angular_velocity.z;
    }

    void operator/=(double factor){
        lin_x /= factor;
        lin_y /= factor;
        lin_z /= factor;
        ang_x /= factor;
        ang_y /= factor;
        ang_z /= factor;
    }

    void operator+=(const imu_data& data){
        lin_x += data.lin_x;
        lin_y += data.lin_y;
        lin_z += data.lin_z;
        ang_x += data.ang_x;
        ang_y += data.ang_y;
        ang_z += data.ang_z;
    }

    void operator-=(const imu_data& data){
        lin_x -= data.lin_x;
        lin_y -= data.lin_y;
        lin_z -= data.lin_z;
        ang_x -= data.ang_x;
        ang_y -= data.ang_y;
        ang_z -= data.ang_z;
    }
} bias_;

bool imu_msg2data(const sensor_msgs::Imu& msg, imu_data &data){
    data.lin_x = msg.linear_acceleration.x;
    data.lin_y = msg.linear_acceleration.y;
    data.lin_z = msg.linear_acceleration.z;
    data.ang_x = msg.angular_velocity.x;
    data.ang_y = msg.angular_velocity.y;
    data.ang_z = msg.angular_velocity.z;
}

std::vector<imu_data> imu_data_vec_(200);

unsigned int imu_init_num_ = 200, imu_init_cnt_ = 0;

void cloud_cb (const sensor_msgs::PointCloud2& input)
{
    if (is_filter_stationary_){
        if (!is_imu_init_)
            return;
        for (auto iter = imu_data_vec_.begin(); iter != imu_data_vec_.end(); iter++){
            if (iter->lin_x > 0.2 || iter->lin_y > 0.2 || iter->lin_z > 0.2
                || iter->ang_x > 0.2 || iter->ang_y > 0.2 || iter->ang_z > 0.2){
                return;
            }
        }
    }

    ROS_INFO("[PC_TRANS] Got one stationary Point Cloud!");
    sensor_msgs::PointCloud2 pc2_msg_out;
    pcl_ros::transformPointCloud("map", input, pc2_msg_out, *listener_ptr_);
    pc2_msg_pub_.publish(pc2_msg_out);
    imu_data_vec_.clear();
}

void imu_cb (const sensor_msgs::Imu& msg)
{
    if (imu_init_cnt_ < imu_init_num_){
        imu_data imu_data_init(msg);
        imu_data_init /= 200;
        bias_ += imu_data_init;
        imu_init_cnt_ ++;
        return;
    }
    if (imu_init_cnt_ == imu_init_num_){
        ROS_WARN_STREAM("bias:" << bias_.lin_x << ", " << bias_.lin_y << ", " << bias_.lin_z << ", " << bias_.ang_x << ", " << bias_.ang_y << ", " << bias_.ang_z);
        is_imu_init_ = true;
        imu_init_cnt_++;
    }
    imu_data data(msg);
    data-=bias_;
    imu_data_vec_.push_back(data);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_transformer");
    ros::NodeHandle nh("~");
    listener_ptr_ = new (tf::TransformListener);
    nh.param<bool>("is_filter_stationary", is_filter_stationary_, true);
    ros::Subscriber pc_sub  = nh.subscribe("/velodyne_points", 50, &cloud_cb);
    ros::Subscriber imu_sub = nh.subscribe("/imu0", 200, &imu_cb);
    pc2_msg_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_world", 50);
    ros::spin();
}