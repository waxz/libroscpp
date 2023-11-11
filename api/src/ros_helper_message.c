//
// Created by waxz on 11/3/23.
//

#include "ros_helper_message.h"


STRUCT_NEW(MemPool);

void* MemPool_set_nodes(MemPool* t, u32 n){
//    MLOGI("%s","MemPool_set_nodes");

    if(t->nodes){
        t->nodes = (void**)ta_realloc(t->nodes, n* sizeof (void*));
    }else{
        t->nodes = (void**)ta_calloc(n, sizeof (void*));
    }
    if(t->nodes ){
        memset(t->nodes + t->node_len, 0, (n - t->node_len) * sizeof(void*));
        t->node_len = n;
    }
    return t->nodes;
#if 0
    for(uint16_t i = t->node_len ; i < n; i++){
        t->nodes[i].count = 0;
        t->nodes[i].ptr = 0;
    }
#endif
}



void* MemPool_reset_counter(MemPool* t){
    t->valid_len = 0;
    return 0;
}



// allocate or reuse existed pointer
void* MemPool_get(MemPool* t, void* s ){
    if(!t || !t->nodes || !s){
        return 0;
    }
//    MLOGI("%s","MemPool_get");
    StructBase* sample =(StructBase*) s;

    bool found = false;
    for(uint16_t i = t->valid_len ; i < t->node_len; i++){

        if(t->nodes[i]){
//            MLOGI("%s","MemPool_get");

            t->nodes[i] = ta_realloc(t->nodes[i], sample->base_size + sample->buffer_size);
            t->valid_len++;
            memcpy(t->nodes[i], sample,sample->base_size);

            return t->nodes[i];
        }else{
//            MLOGI("%s","MemPool_get");

            t->nodes[i] = ta_alloc(sample->base_size + sample->buffer_size);
            memcpy(t->nodes[i], sample,sample->base_size);
            t->valid_len++;

            return t->nodes[i];
        }
    }
//    MLOGI("%s","MemPool_get");

    uint16_t old_len = t->node_len;
    MemPool_set_nodes(t, t->node_len+5);
    // create new
    uint16_t new_index = old_len;
    t->nodes[new_index] = ta_alloc(sample->base_size + sample->buffer_size);
    memcpy(t->nodes[new_index], sample,sample->base_size);
    t->valid_len++;
    return t->nodes[new_index] ;
}


STRUCT_NEW(LaserScanT);

void LaserScanT_set_buffer(LaserScanT* t, u32 size){
    t->ranges_size = size;
    t->buffer_size = (size + size)*sizeof(f32) ;
}
LaserScanT_ptr LaserScanT_alloc( u32 size){
    LaserScanT target = LaserScanT_create();
    LaserScanT_set_buffer(&target,size);
    LaserScanT_ptr ptr = ta_alloc(target.base_size + target.buffer_size);
    *ptr = target;
    return ptr;
}

float* LaserScanT_get_ranges(LaserScanT* t){
    return t->buffer;
}
float* LaserScanT_get_intensities(LaserScanT* t){
    return t->buffer + t->ranges_size;
}

STRUCT_NEW(TwistT);


TwistT_ptr TwistT_alloc(u32 size){
    TwistT target = TwistT_create();
    TwistT_ptr ptr = ta_alloc(target.base_size + target.buffer_size);
    *ptr = target;
    return ptr;
}


STRUCT_NEW(PoseStampedT);
PoseStampedT_ptr PoseStampedT_alloc(u32 size){
    PoseStampedT target = PoseStampedT_create();
    PoseStampedT_ptr ptr = ta_alloc(target.base_size + target.buffer_size);
    *ptr = target;
    return ptr;
}

STRUCT_NEW(PoseT);


STRUCT_NEW(UInt8MultiArrayT);
void UInt8MultiArrayT_set_buffer(UInt8MultiArrayT* t, u32 size){
    t->buffer_size = size * sizeof (u8);
    t->element_size = size;
}
u8* UInt8MultiArrayT_get_buffer(UInt8MultiArrayT* t){
    return t->buffer;
}
UInt8MultiArrayT_ptr UInt8MultiArrayT_alloc(u32 size){
    UInt8MultiArrayT target = UInt8MultiArrayT_create();
    UInt8MultiArrayT_set_buffer(&target,size);
    UInt8MultiArrayT_ptr ptr = ta_alloc(target.base_size + target.buffer_size);
    *ptr = target;
    return ptr;
}




STRUCT_NEW(UInt16MultiArrayT);
void UInt16MultiArrayT_set_buffer(UInt16MultiArrayT* t, u32 size){
    t->buffer_size = size * sizeof (u16);
    t->element_size = size;
}
UInt16MultiArrayT_ptr UInt16MultiArrayT_alloc(u32 size){
    UInt16MultiArrayT target = UInt16MultiArrayT_create();
    UInt16MultiArrayT_set_buffer(&target,size);
    UInt16MultiArrayT_ptr ptr = ta_alloc(target.base_size + target.buffer_size);
    *ptr = target;
    return ptr;
}
u16* UInt16MultiArrayT_get_buffer(UInt16MultiArrayT* t){
    return t->buffer;
}

STRUCT_NEW(OdometryT)
OdometryT_ptr OdometryT_alloc(u32 size){
    OdometryT target = OdometryT_create();
    OdometryT_ptr ptr = ta_alloc(target.base_size + target.buffer_size);
    *ptr = target;
    return ptr;
}

STRUCT_NEW(OccupancyGridT)
void OccupancyGridT_set_buffer(OccupancyGridT* t,u32 width, u32 height){
    t->width = width;
    t->height = height;
    t->buffer_size = width*height*sizeof(i8);
}

OccupancyGridT_ptr OccupancyGridT_alloc(u32 width, u32 height){
    OccupancyGridT target = OccupancyGridT_create();
    OccupancyGridT_set_buffer(&target,width , height);
    OccupancyGridT_ptr ptr = ta_alloc(target.base_size + target.buffer_size);
    *ptr = target;
    return ptr;
}


STRUCT_NEW(ChannelBufferT)