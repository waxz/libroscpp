//
// Created by waxz on 9/13/23.
//




// toml
#include <toml.hpp>
// ros
#include "ros/ros.h"
#include <ros/callback_queue.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include "ros_helper.h"
#include "ros_helper_impl.h"

#include "common/data_holder.h"

// plain data type
#ifdef __cplusplus

extern "C" {

#endif


#ifdef __cplusplus
};
#endif


//http://wiki.ros.org/Remapping%20Arguments


void ros_create_from_toml(ros_handler_ptr_t h, const char* filename){
    RosHandler* handler = new RosHandler();
    int rt = handler->create_from_toml(filename);
    h->handler = handler;
    MLOGI("%s ret = %i\n","ros_create_from_toml",rt);
    if(rt < 0 ){
        h->close(h);
    }

}

void ros_close(ros_handler_ptr_t h){
    MLOGI("%s h->handler = %p\n","ros_close",h->handler);
    if(h->handler){
        RosHandler* handler = (RosHandler*)h->handler;
        handler->stop();
        MLOGI("ros_close at : %p\n",h->handler);
        delete handler;
        h->handler = nullptr;
    }

}
bool ros_is_ok(struct ros_handler_t* h){

    return h->handler != nullptr && ros::ok();
}
int ros_read(ros_handler_ptr_t h, const char* channel_name){
    if(h->handler){
        RosHandler* handler = (RosHandler*)h->handler;
        return handler->read(channel_name);
    }
    return 0;
}
int ros_write(ros_handler_ptr_t h, const char* channel_name){
    if(h->handler){
        RosHandler* handler = (RosHandler*)h->handler;
        return handler->write(channel_name);
    }
    return 0;
}
int ros_write_another_pool(ros_handler_ptr_t h, const char* channel_name,MemPool_ptr pool){
    if(h->handler){
        RosHandler* handler = (RosHandler*)h->handler;
        return handler->write(channel_name,pool);
    }
    return 0;
}
MemPool_ptr ros_get_pool(struct ros_handler_t* h, const char* channel_name){
    if(h->handler){
        RosHandler* handler = (RosHandler*)h->handler;

        return handler->get_pool(channel_name);
    }
    return 0;
}
ros_handler_t ros_handler_t_create(){
    ros_handler_t target = {0};

    target.create = ros_create_from_toml;
    target.close = ros_close;
    target.read = ros_read;
    target.write = ros_write;
//    target.pool = ros_get_pool;
    target.is_ok = ros_is_ok;
    target.read_data = ros_read_data;
    target.write_data = ros_write_data;

    return target;
}

int ros_write_data (ros_handler_ptr_t h, const char* channel_name, void** buffer, u32 buffer_size){
    if(h->handler){
        RosHandler* handler = (RosHandler*)h->handler;
        return handler->write_data(channel_name,buffer,buffer_size);
    }
    return 0;
}
ChannelBufferT_ptr ros_read_data (ros_handler_ptr_t h, const char* channel_name){
    if(h->handler){
        RosHandler* handler = (RosHandler*)h->handler;
        return handler->read_data(channel_name );
    }
    return 0;
}