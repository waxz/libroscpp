//
// Created by waxz on 10/19/23.
//

#include "ros_helper_topic.h"
#include <string>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <nav_msgs/OccupancyGrid.h>

#include "ros_helper_message.h"


//LaserScan
int from_common_LaserScan(ros::Publisher& publisher,void** buffer, uint32_t buffer_size);

void to_common_LaserScan(sensor_msgs::LaserScanConstPtr msg, MemPool_ptr pool);

ROSTopicWriter create_writer_LaserScan(ros::NodeHandle& nh, writer_option* option);

ROSTopicReader create_reader_LaserScan(ros::NodeHandle& nh, reader_option* option);

//Twist
int from_common_Twist(ros::Publisher& publisher, void** buffer, uint32_t buffer_size);

ROSTopicWriter create_writer_Twist(ros::NodeHandle& nh, writer_option* option);

void to_common_Twist(geometry_msgs::TwistConstPtr msg, MemPool_ptr pool);

ROSTopicReader create_reader_Twist(ros::NodeHandle& nh, reader_option* option);

//UInt8MultiArray
int from_common_UInt8MultiArray(ros::Publisher& publisher, void** buffer, uint32_t buffer_size);

ROSTopicWriter create_writer_UInt8MultiArray(ros::NodeHandle& nh, writer_option* option);

void to_common_UInt8MultiArray(std_msgs::UInt8MultiArrayConstPtr msg, MemPool_ptr pool);

ROSTopicReader create_reader_UInt8MultiArray(ros::NodeHandle& nh, reader_option* option);

//UInt16MultiArray
int from_common_UInt16MultiArray(ros::Publisher& publisher, void** buffer, uint32_t buffer_size);

ROSTopicWriter create_writer_UInt16MultiArray(ros::NodeHandle& nh, writer_option* option);

void to_common_UInt16MultiArray(std_msgs::UInt16MultiArrayConstPtr msg, MemPool_ptr pool);

ROSTopicReader create_reader_UInt16MultiArray(ros::NodeHandle& nh, reader_option* option);

//OccupancyGrid
int from_common_OccupancyGrid(ros::Publisher& publisher, void** buffer, uint32_t buffer_size);

ROSTopicWriter create_writer_OccupancyGrid(ros::NodeHandle& nh, writer_option* option);

void to_common_OccupancyGrid(nav_msgs::OccupancyGridConstPtr msg, MemPool_ptr pool);

ROSTopicReader create_reader_OccupancyGrid(ros::NodeHandle& nh, reader_option* option);

// Odometry
int from_common_Odometry(ros::Publisher& publisher, void** buffer, uint32_t buffer_size);

ROSTopicWriter create_writer_Odometry(ros::NodeHandle& nh, writer_option* option);

void to_common_Odometry(nav_msgs::OdometryConstPtr msg, MemPool_ptr pool);

ROSTopicReader create_reader_Odometry(ros::NodeHandle& nh, reader_option* option);




// LaserScan

int from_common_LaserScan(ros::Publisher& publisher, void** buffer, uint32_t buffer_size){

    static sensor_msgs::LaserScan msg;

    msg.header.stamp = ros::Time::now();

    for(int i = 0; i < buffer_size ;i++){
        LaserScanT* scan_ptr =  (LaserScanT* )buffer[i] ;

        msg.header.frame_id.assign(scan_ptr->frame_id);
        msg.range_min = scan_ptr->range_min;
        msg.range_max = scan_ptr->range_max;

        msg.angle_min = scan_ptr->angle_min;
        msg.angle_max = scan_ptr->angle_max;
        msg.angle_increment = scan_ptr->angle_increment;

        msg.ranges.resize(scan_ptr->ranges_size);
        msg.intensities.resize(scan_ptr->ranges_size);

        memcpy( msg.ranges.data(),LaserScanT_get_ranges(scan_ptr), scan_ptr->ranges_size*sizeof (float));
        memcpy( msg.intensities.data(),LaserScanT_get_intensities(scan_ptr), scan_ptr->ranges_size*sizeof (float));

        publisher.publish(msg);
    }


    return 0;
}
void to_common_LaserScan(sensor_msgs::LaserScanConstPtr msg, MemPool_ptr pool){

    LaserScanT scan = LaserScanT_create();
    size_t ranges_size = msg->ranges.size();
    LaserScanT_set_buffer(&scan, ranges_size);

    LaserScanT* scan_ptr =  (LaserScanT*)MemPool_get(pool,&scan);
    // frame_id
    strcpy(scan_ptr->frame_id,msg->header.frame_id.c_str());

    // basic info
    scan_ptr->range_min = msg->range_min;
    scan_ptr->range_max = msg->range_max;
    scan_ptr->angle_min = msg->angle_min;
    scan_ptr->angle_max = msg->angle_max;
    scan_ptr->angle_increment = msg->angle_increment;


    // ranges

    memcpy(LaserScanT_get_ranges(scan_ptr), msg->ranges.data(),scan_ptr->ranges_size*sizeof (float));
    if(!msg->intensities.empty()){
        memcpy(LaserScanT_get_intensities(scan_ptr), msg->intensities.data(),scan_ptr->ranges_size*sizeof (float));
    }

}

ROSTopicWriter create_writer_LaserScan(ros::NodeHandle& nh, writer_option* option){
    ROSTopicWriter target;
    target.valid = true;

    target.writer =nh.advertise<sensor_msgs::LaserScan>(option->topic_name, option->queue_size,option->keep_last);
     target.write_data = from_common_LaserScan;

    return target;
}

ROSTopicReader create_reader_LaserScan(ros::NodeHandle& nh, reader_option* option){
    ROSTopicReader target;
    target.valid = true;

    target.reader = nh.subscribe<sensor_msgs::LaserScan>(option->topic_name, option->queue_size, [option](sensor_msgs::LaserScanConstPtr msg){

        to_common_LaserScan(msg, option->pool);
    });

    return target;
}

// Twist
int from_common_Twist(ros::Publisher& publisher, void** buffer, uint32_t buffer_size){

    static geometry_msgs::Twist msg;

    for(int i = 0; i < buffer_size;i++){
        TwistT* ptr =  (TwistT* )buffer[i] ;

        msg.angular.x = ptr->angular.x;
        msg.angular.y = ptr->angular.y;
        msg.angular.z = ptr->angular.z;

        msg.linear.x = ptr->linear.x;
        msg.linear.y = ptr->linear.y;
        msg.linear.z = ptr->linear.z;

        publisher.publish(msg);
    }
    return 0;
}

ROSTopicWriter create_writer_Twist(ros::NodeHandle& nh, writer_option* option){
    ROSTopicWriter target;
    target.valid = true;

    target.writer =nh.advertise<geometry_msgs::Twist>(option->topic_name, option->queue_size,option->keep_last);
    target.write_data = from_common_Twist;

    return target;
}

void to_common_Twist(geometry_msgs::TwistConstPtr msg, MemPool_ptr pool){

    TwistT tem_target = TwistT_create();
    TwistT* ptr_target =  (TwistT*)MemPool_get(pool,&tem_target);


    ptr_target->angular.x =  msg->angular.x;
    ptr_target->angular.y =  msg->angular.y;
    ptr_target->angular.z =  msg->angular.z;

    ptr_target->linear.x =  msg->linear.x;
    ptr_target->linear.y =  msg->linear.y;
    ptr_target->linear.z =  msg->linear.z;

}

ROSTopicReader create_reader_Twist(ros::NodeHandle& nh, reader_option* option){
    ROSTopicReader target;
    target.valid = true;

    target.reader = nh.subscribe<geometry_msgs::Twist>(option->topic_name, option->queue_size, [option](geometry_msgs::TwistConstPtr msg){

        to_common_Twist(msg, option->pool);
    });

    return target;
}


// Odometry


int from_common_Odometry(ros::Publisher& publisher, void** buffer, uint32_t buffer_size){
    static nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time::now();

    for(int i = 0; i < buffer_size;i++){
        OdometryT * ptr =  (OdometryT * )buffer[i] ;

        msg.header.frame_id.assign(ptr->frame_id);
        msg.child_frame_id.assign(ptr->child_frame_id);


        msg.pose.pose.position.x = ptr->pose.position.x;
        msg.pose.pose.position.y = ptr->pose.position.y;
        msg.pose.pose.position.z = ptr->pose.position.z;

        msg.pose.pose.orientation.w = ptr->pose.quaternion.w;
        msg.pose.pose.orientation.x = ptr->pose.quaternion.x;
        msg.pose.pose.orientation.y = ptr->pose.quaternion.y;
        msg.pose.pose.orientation.z = ptr->pose.quaternion.z;

        std::memcpy(msg.pose.covariance.data(), ptr->pose_cov, 36*sizeof(double ) );


        msg.twist.twist.linear.x = ptr->twist.linear.x;
        msg.twist.twist.linear.y = ptr->twist.linear.y;
        msg.twist.twist.linear.z = ptr->twist.linear.z;

        msg.twist.twist.angular.x = ptr->twist.angular.x;
        msg.twist.twist.angular.y = ptr->twist.angular.y;
        msg.twist.twist.angular.z = ptr->twist.angular.z;

        std::memcpy(msg.twist.covariance.data(), ptr->twist_cov, 36*sizeof(double ) );

        publisher.publish(msg);
    }
    return 0;
}

ROSTopicWriter create_writer_Odometry(ros::NodeHandle& nh, writer_option* option){
    ROSTopicWriter target;
    target.valid = true;

    target.writer =nh.advertise<nav_msgs::Odometry>(option->topic_name, option->queue_size,option->keep_last);
    target.write_data = from_common_Odometry;

    return target;
}

void to_common_Odometry(nav_msgs::OdometryConstPtr msg, MemPool_ptr pool){

    OdometryT tem_target = OdometryT_create();
    OdometryT* ptr_target =  (OdometryT*)MemPool_get(pool,&tem_target);

    std::strcpy(ptr_target->frame_id, msg->header.frame_id.c_str());

    std::strcpy(ptr_target->child_frame_id, msg->child_frame_id.c_str());

    ptr_target->pose.position.x =  msg->pose.pose.position.x;
    ptr_target->pose.position.y =  msg->pose.pose.position.y;
    ptr_target->pose.position.z =  msg->pose.pose.position.z;

    ptr_target->pose.quaternion.w =  msg->pose.pose.orientation.w;
    ptr_target->pose.quaternion.x =  msg->pose.pose.orientation.x;
    ptr_target->pose.quaternion.y =  msg->pose.pose.orientation.y;
    ptr_target->pose.quaternion.z =  msg->pose.pose.orientation.z;
    std::memcpy(ptr_target->pose_cov, msg->pose.covariance.data(), 36*sizeof(double ) );

    ptr_target->twist.linear.x = msg->twist.twist.linear.x;
    ptr_target->twist.linear.y = msg->twist.twist.linear.y;
    ptr_target->twist.linear.z = msg->twist.twist.linear.z;

    ptr_target->twist.angular.x = msg->twist.twist.angular.x;
    ptr_target->twist.angular.y = msg->twist.twist.angular.y;
    ptr_target->twist.angular.z = msg->twist.twist.angular.z;

    std::memcpy(ptr_target->twist_cov, msg->twist.covariance.data(), 36*sizeof(double) );
}

ROSTopicReader create_reader_Odometry(ros::NodeHandle& nh, reader_option* option){
    ROSTopicReader target;
    target.valid = true;

    target.reader = nh.subscribe<nav_msgs::Odometry>(option->topic_name, option->queue_size, [option](nav_msgs::OdometryConstPtr msg){

        to_common_Odometry(msg, option->pool);
    });

    return target;
}




// UInt8MultiArray

int from_common_UInt8MultiArray(ros::Publisher& publisher, void** buffer, uint32_t buffer_size){

    static std_msgs::UInt8MultiArray msg;

    for(int i = 0; i < buffer_size;i++){
        UInt8MultiArrayT* ptr =  (UInt8MultiArrayT* )buffer[i] ;
        msg.data.resize(ptr->element_size);
        memcpy(msg.data.data(), ptr->buffer, ptr->buffer_size);
        publisher.publish(msg);
    }
    return 0;
}

ROSTopicWriter create_writer_UInt8MultiArray(ros::NodeHandle& nh, writer_option* option){
    ROSTopicWriter target;
    target.valid = true;

    target.writer =nh.advertise<std_msgs::UInt8MultiArray>(option->topic_name, option->queue_size,option->keep_last);
    target.write_data = from_common_UInt8MultiArray;
    return target;
}

void to_common_UInt8MultiArray(std_msgs::UInt8MultiArrayConstPtr msg, MemPool_ptr pool){

    UInt8MultiArrayT tem_target = UInt8MultiArrayT_create();
    UInt8MultiArrayT_set_buffer(&tem_target,msg->data.size());
    UInt8MultiArrayT* ptr_target =  (UInt8MultiArrayT*)MemPool_get(pool,&tem_target);
    memcpy(ptr_target->buffer, msg->data.data(),ptr_target->buffer_size );
}


ROSTopicReader create_reader_UInt8MultiArray(ros::NodeHandle& nh, reader_option* option){
    ROSTopicReader target;
    target.valid = true;

    target.reader = nh.subscribe<std_msgs::UInt8MultiArray>(option->topic_name, option->queue_size, [option](std_msgs::UInt8MultiArrayConstPtr msg){

        to_common_UInt8MultiArray(msg, option->pool);
    });

    return target;
}

// UInt16MultiArray
int from_common_UInt16MultiArray(ros::Publisher& publisher, void** buffer, uint32_t buffer_size){

    static std_msgs::UInt16MultiArray msg;

    for(int i = 0; i < buffer_size;i++){
        UInt16MultiArrayT* ptr =  (UInt16MultiArrayT* )buffer[i] ;
        msg.data.resize(ptr->element_size);
        memcpy(msg.data.data(), ptr->buffer, ptr->buffer_size);
        publisher.publish(msg);
    }
    return 0;
}

ROSTopicWriter create_writer_UInt16MultiArray(ros::NodeHandle& nh, writer_option* option){
    ROSTopicWriter target;
    target.valid = true;

    target.writer =nh.advertise<std_msgs::UInt16MultiArray>(option->topic_name, option->queue_size,option->keep_last);
    target.write_data = from_common_UInt16MultiArray;
    return target;
}

void to_common_UInt16MultiArray(std_msgs::UInt16MultiArrayConstPtr msg, MemPool_ptr pool){

    UInt16MultiArrayT tem_target = UInt16MultiArrayT_create();
    UInt16MultiArrayT_set_buffer(&tem_target,msg->data.size());

    UInt16MultiArrayT* ptr_target =  (UInt16MultiArrayT*)MemPool_get(pool,&tem_target);

    memcpy(ptr_target->buffer, msg->data.data(),ptr_target->buffer_size );
}

ROSTopicReader create_reader_UInt16MultiArray(ros::NodeHandle& nh, reader_option* option){
    ROSTopicReader target;
    target.valid = true;

    target.reader = nh.subscribe<std_msgs::UInt16MultiArray>(option->topic_name, option->queue_size, [option](std_msgs::UInt16MultiArrayConstPtr msg){

        to_common_UInt16MultiArray(msg, option->pool);
    });

    return target;
}

//===============
int from_common_OccupancyGrid(ros::Publisher& publisher, void** buffer, uint32_t buffer_size){
    static nav_msgs::OccupancyGrid msg;
    msg.header.stamp = ros::Time::now();

    for(int i = 0; i < buffer_size;i++){
        OccupancyGridT* ptr =  (OccupancyGridT* )buffer[i] ;

        msg.header.frame_id.assign(ptr->frame_id);
        msg.info.width = ptr->width;
        msg.info.height = ptr->height;
        msg.info.resolution = ptr->resolution;


        msg.info.origin.position.x = ptr->origin.position.x;
        msg.info.origin.position.y = ptr->origin.position.y;
        msg.info.origin.position.z = ptr->origin.position.z;

        msg.info.origin.orientation.w = ptr->origin.quaternion.w;
        msg.info.origin.orientation.x = ptr->origin.quaternion.x;
        msg.info.origin.orientation.y = ptr->origin.quaternion.y;
        msg.info.origin.orientation.z = ptr->origin.quaternion.z;

        msg.data.resize(ptr->width * ptr->height);
        memcpy(msg.data.data(), ptr->data, ptr->buffer_size);
        publisher.publish(msg);
    }
    return 0;
}

ROSTopicWriter create_writer_OccupancyGrid(ros::NodeHandle& nh, writer_option* option){
    ROSTopicWriter target;
    target.valid = true;

    target.writer =nh.advertise<nav_msgs::OccupancyGrid>(option->topic_name, option->queue_size,option->keep_last);
    target.write_data = from_common_OccupancyGrid;
    return target;
}

void to_common_OccupancyGrid(nav_msgs::OccupancyGridConstPtr msg, MemPool_ptr pool){

    OccupancyGridT tem_target = OccupancyGridT_create();
    OccupancyGridT_set_buffer(&tem_target,msg->info.width, msg->info.height);
    OccupancyGridT* ptr_target =  (OccupancyGridT*)MemPool_get(pool,&tem_target);

    //
    strcpy(ptr_target->frame_id, msg->header.frame_id.c_str());
    ptr_target->width = msg->info.width;
    ptr_target->height = msg->info.height;
    ptr_target->resolution = msg->info.resolution;

    ptr_target->origin.position.x = msg->info.origin.position.x;
    ptr_target->origin.position.y = msg->info.origin.position.y;
    ptr_target->origin.position.z = msg->info.origin.position.z;

    ptr_target->origin.quaternion.w = msg->info.origin.orientation.w;
    ptr_target->origin.quaternion.x = msg->info.origin.orientation.x;
    ptr_target->origin.quaternion.y = msg->info.origin.orientation.y;
    ptr_target->origin.quaternion.z = msg->info.origin.orientation.z;

    memcpy(ptr_target->data, msg->data.data(),ptr_target->buffer_size );
}

ROSTopicReader create_reader_OccupancyGrid(ros::NodeHandle& nh, reader_option* option){
    ROSTopicReader target;
    target.valid = true;

    target.reader = nh.subscribe<nav_msgs::OccupancyGrid>(option->topic_name, option->queue_size, [option](nav_msgs::OccupancyGridConstPtr msg){
        to_common_OccupancyGrid(msg, option->pool);
    });

    return target;
}

//================

ROSTopicReader create_reader_Pose(ros::NodeHandle& nh, reader_option* option){
    ROSTopicReader target;
    return target;
}

ROSTopicReader create_reader_Path(ros::NodeHandle& nh, reader_option* option){
    ROSTopicReader target;
    return target;
}



ROSTopicReader create_reader(ros::NodeHandle& nh, reader_option* option){


    if(std::strcmp(option->topic_type,"LaserScan") == 0 ){
        return create_reader_LaserScan(nh,option);
    } else if(std::strcmp(option->topic_type,"Twist") == 0 ){
        return create_reader_Twist(nh,option);
    }else if(std::strcmp(option->topic_type,"UInt8MultiArray") ==0){
        return create_reader_UInt8MultiArray(nh,option);

    }else if(std::strcmp(option->topic_type,"UInt16MultiArray") ==0){
        return create_reader_UInt16MultiArray(nh,option);

    }else if(std::strcmp(option->topic_type,"OccupancyGrid") ==0){
        return create_reader_OccupancyGrid(nh,option);

    }else if(std::strcmp(option->topic_type,"Odometry") ==0){
        return create_reader_Odometry(nh,option);

    }

    ROSTopicReader reader;
    reader.valid = false;
    return reader;

}

ROSTopicWriter create_writer(ros::NodeHandle& nh, writer_option* option){

    if(std::strcmp(option->topic_type,"LaserScan") == 0 ){
        return create_writer_LaserScan(nh,option);
    } else if(std::strcmp(option->topic_type,"Twist") == 0 ){
        return create_writer_Twist(nh,option);
    }else if(std::strcmp(option->topic_type,"UInt8MultiArray") ==0){
        return create_writer_UInt8MultiArray(nh,option);

    }else if(std::strcmp(option->topic_type,"UInt16MultiArray") ==0){
        return create_writer_UInt16MultiArray(nh,option);
    }else if(std::strcmp(option->topic_type,"OccupancyGrid") ==0){
        return create_writer_OccupancyGrid(nh,option);
    }else if(std::strcmp(option->topic_type,"Odometry") ==0){
        return create_writer_Odometry(nh,option);
    }





    ROSTopicWriter reader;
    reader.valid = false;
    return reader;
}


int ros_write_tf(std::shared_ptr<tf::TransformBroadcaster> tfb, tf::StampedTransform& target, MemPool_ptr pool ){

    for(int i = 0; i < pool->valid_len;i++){
//        if(pool->nodes[i].ptr == nullptr || pool->nodes[i].count ==0){ break;}

        PoseStampedT_ptr data = (PoseStampedT_ptr)pool->nodes[i];
        target.setOrigin(tf::Vector3(data->position.x,
                                     data->position.y,
                                     data->position.z));
        target.setRotation(tf::Quaternion(data->quaternion.x,
                                          data->quaternion.y,
                                          data->quaternion.z,
                                          data->quaternion.w));

        tfb->sendTransform(target);

    }
    return 0;

}
int ros_write_tf_data(std::shared_ptr<tf::TransformBroadcaster> tfb, tf::StampedTransform& target, void** buffer, uint32_t buffer_size ){

    for(int i = 0; i < buffer_size;i++){
//        if(pool->nodes[i].ptr == nullptr || pool->nodes[i].count ==0){ break;}

        PoseStampedT_ptr data = (PoseStampedT_ptr)buffer[i];
        target.setOrigin(tf::Vector3(data->position.x,
                                     data->position.y,
                                     data->position.z));
        target.setRotation(tf::Quaternion(data->quaternion.x,
                                          data->quaternion.y,
                                          data->quaternion.z,
                                          data->quaternion.w));

        tfb->sendTransform(target);

    }
    return 0;
}

int ros_read_tf(std::shared_ptr<tf::TransformListener> tfl, tf::StampedTransform& target, MemPool_ptr pool ){

    tfl->lookupTransform(target.frame_id_, target.child_frame_id_,ros::Time(0), target);


    static PoseStampedT sample = PoseStampedT_create();
    sample.position.x = target.getOrigin().x();
    sample.position.y = target.getOrigin().y();
    sample.position.z = target.getOrigin().z();

    sample.quaternion.w = target.getRotation().w();
    sample.quaternion.x = target.getRotation().x();
    sample.quaternion.y = target.getRotation().y();
    sample.quaternion.z = target.getRotation().z();
    MemPool_get(pool,&sample);

    return 0;
}