//
// Created by waxz on 10/19/23.
//

#ifndef CMAKE_SUPER_BUILD_ROS_HELPER_MESSAGE_H
#define CMAKE_SUPER_BUILD_ROS_HELPER_MESSAGE_H

#include "common/data_holder.h"

#ifdef __cplusplus
extern "C" {
#endif




STRUCT_BEGIN(StructBase)
STRUCT_END(StructBase)

// memory pool size should be fixed or dynamic
// allocate function should return absolute address or relative address
// reserve buffer, avoid dynamic

STRUCT_BEGIN(MemPool)

    // all node should keep in nodes
    void** nodes;
    u32 node_len;
    u32 valid_len;

STRUCT_END(MemPool)
void* MemPool_set_nodes(MemPool* t, u32 n);
void* MemPool_reset_counter(MemPool* t);
void* MemPool_get(MemPool* t, void* s );
// inline function
//static inline void* mem_pool_set_nodes(MemPool* t, u32 n);




STRUCT_BEGIN(LaserScanT)
    char frame_id[50];
    size_t stamp;
    f32 range_min;
    f32 range_max;
    f32 angle_min;
    f32 angle_max;
    f32 angle_increment;
    u32 ranges_size;
    f32 buffer[1];
STRUCT_END(LaserScanT)
LaserScanT_ptr LaserScanT_alloc( u32 size);
void LaserScanT_set_buffer(LaserScanT* t, u32 size);

f32* LaserScanT_get_ranges(LaserScanT* t);
f32* LaserScanT_get_intensities(LaserScanT* t);


STRUCT_BEGIN(PointT)
    f64 x;
    f64 y;
    f64 z;
STRUCT_END(PointT)

STRUCT_BEGIN(QuaternionT)
    f64 w;
    f64 x;
    f64 y;
    f64 z;
STRUCT_END(QuaternionT)


STRUCT_BEGIN(PoseT)
    PointT position;
    QuaternionT quaternion;
STRUCT_END(PoseT)

STRUCT_BEGIN(PoseStampedT)
    size_t stamp;
    PointT position;
    QuaternionT quaternion;
STRUCT_END(PoseStampedT)

PoseStampedT_ptr PoseStampedT_alloc(u32 size);


STRUCT_BEGIN(TwistT)
    PointT angular;
    PointT linear;
STRUCT_END(TwistT)
TwistT_ptr TwistT_alloc(u32 size);

STRUCT_BEGIN(UInt8MultiArrayT)
    u32 element_size;
    u8 buffer[1];
STRUCT_END(UInt8MultiArrayT)

UInt8MultiArrayT_ptr UInt8MultiArrayT_alloc(u32 size);

void UInt8MultiArrayT_set_buffer(UInt8MultiArrayT* t, u32 size);
u8* UInt8MultiArrayT_get_buffer(UInt8MultiArrayT* t);
STRUCT_BEGIN(UInt16MultiArrayT)
    u32 element_size;
    u16 buffer[1];
STRUCT_END(UInt16MultiArrayT)
UInt16MultiArrayT_ptr UInt16MultiArrayT_alloc(u32 size);
void UInt16MultiArrayT_set_buffer(UInt16MultiArrayT* t, u32 size);
u16* UInt16MultiArrayT_get_buffer(UInt16MultiArrayT* t);


//Odometry
STRUCT_BEGIN(OdometryT)
    char frame_id[50];
    char child_frame_id[50];
    PoseT pose;
    f64 pose_cov[36];
    TwistT twist;
    f64 twist_cov[36];
STRUCT_END(OdometryT)

OdometryT_ptr OdometryT_alloc(u32 size);


STRUCT_BEGIN(OccupancyGridT)
    char frame_id[50];
    f32 resolution;
    u32 width;
    u32 height;
    PoseT origin;
    i8 data[1];
STRUCT_END(OccupancyGridT)

OccupancyGridT_ptr OccupancyGridT_alloc(u32 width, u32 height);
void OccupancyGridT_set_buffer(OccupancyGridT* t,u32 width, u32 height);



STRUCT_BEGIN(ChannelBufferT)

    // array  of pointer, dynamic allocate
    void** buffer;
    u32 len;
STRUCT_END(ChannelBufferT)

#ifdef __cplusplus
}
#endif

#endif //CMAKE_SUPER_BUILD_ROS_HELPER_MESSAGE_H
