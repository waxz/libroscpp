//
// Created by waxz on 10/19/23.
//

#ifndef CMAKE_SUPER_BUILD_ROS_HELPER_OPTION_H
#define CMAKE_SUPER_BUILD_ROS_HELPER_OPTION_H

#include <stdbool.h>



#ifdef __cplusplus
extern "C" {
#endif


    typedef struct MemPool *MemPool_ptr;

    // qos

typedef struct reader_option{
    char topic_name[200];
    char topic_type[200];

    int queue_size;
    int wait_time_s;

    MemPool_ptr pool;

}reader_option;


typedef struct writer_option{
    char topic_name[200];
    char topic_type[200];

    int queue_size;
    bool keep_last;

    MemPool_ptr pool;

}writer_option;





#ifdef __cplusplus
}
#endif

#endif //CMAKE_SUPER_BUILD_ROS_HELPER_OPTION_H
