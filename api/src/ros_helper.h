//
// Created by waxz on 9/13/23.
//

#ifndef CMAKE_SUPER_BUILD_ROS_HELPER_H
#define CMAKE_SUPER_BUILD_ROS_HELPER_H

//#include "common/data_holder.h"
#include "ros_helper_message.h"

#ifdef __cplusplus
extern "C" {
#endif

// crete
// clean
// is_running
// read_all
// read_channel
// write_all
// write_channel
// write_channel_data



typedef struct ros_handler_t{

    void* handler;

    /// crete ros node and subscriber and publisher from toml file
    /// \param h
    /// \param filename
    void(*create)(struct ros_handler_t* h, const char* filename);
    /// close all resource
    /// \param h
    void(*close)(struct ros_handler_t* h);

    /// get ros::ok
    /// \param h
    /// \return
    bool(*is_ok)(struct ros_handler_t* h);

    /// read data from channel
    /// \param h
    /// \param channel_name
    /// \return
    int(*read)(struct ros_handler_t* h, const char* channel_name);

    /// write data to channel
    /// \param h
    /// \param channel_name
    /// \return
    int(*write)(struct ros_handler_t* h, const char* channel_name);
    int(*write_another_pool)(struct ros_handler_t* h, const char* channel_name, MemPool_ptr pool);

    int(*write_data)(struct ros_handler_t* h, const char* channel_name, void** buffer, u32 buffer_size);

    ChannelBufferT_ptr (*read_data)(struct ros_handler_t* h, const char* channel_name);


    /// get memory pool
    /// \param h
    /// \param channel_name
    /// \return
//    MemPool_ptr(*pool)(struct ros_handler_t* h, const char* channel_name);


}ros_handler_t, *ros_handler_ptr_t;



void ros_create_from_toml(ros_handler_ptr_t h, const char* filename);
void ros_close(ros_handler_ptr_t h);
bool ros_is_ok(struct ros_handler_t* h);


int ros_read(ros_handler_ptr_t h, const char* channel_name);
int ros_write(ros_handler_ptr_t h, const char* channel_name);
int ros_write_another_pool(ros_handler_ptr_t h, const char* channel_name,MemPool_ptr pool);

MemPool_ptr ros_get_pool(ros_handler_ptr_t h, const char* channel_name);


int ros_write_data (ros_handler_ptr_t h, const char* channel_name, void** buffer, u32 buffer_size);
ChannelBufferT_ptr ros_read_data (ros_handler_ptr_t h, const char* channel_name);

ros_handler_t ros_handler_t_create();

#ifdef __cplusplus
};
#endif

#endif //CMAKE_SUPER_BUILD_ROS_HELPER_H
