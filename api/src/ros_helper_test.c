//
// Created by waxz on 9/22/23.
//

#include "ros_helper.h"
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h> // usleep

#include "common/signals.h"

#include "tinyalloc/tinyalloc.h"

#include "ros_helper_message.h"

#include "common/data_holder.h"

static bool program_run = true;

void signal_handler(int sig) {
    printf("get signal %i\n", sig);
    program_run = false;
}



// 10000000 Byte = 10 MB
#define STATIC_MEMORY_SIZE 10000000
static unsigned char memory_pool[STATIC_MEMORY_SIZE];


void test(void ** p){
    *p = NULL;
}

void ta_print(){
    printf("ta_num_used : %lu\n",ta_num_used());
    printf("ta_num_free : %lu\n",ta_num_free());
    printf("ta_num_fresh : %lu\n",ta_num_fresh());
    printf("ta_check : %i\n",ta_check());
}

void test_pool(){

    MemPool pool;
    pool = MemPool_create();

    MemPool_set_nodes(&pool,10);

    {
        LaserScanT scan = LaserScanT_create();
        scan.angle_min = 10;
        scan.angle_max = 100;

        LaserScanT * scan_ptr = MemPool_get(&pool,&scan);
        printf("get:%.3f,%.3f\n",scan_ptr->angle_min,scan_ptr->angle_max );
    }
    {
        PoseStampedT tf_data = PoseStampedT_create();
        tf_data.position.x = 123;
        PoseStampedT * tf_ptr = MemPool_get(&pool,&tf_data);
        printf("get:%.3f\n",tf_ptr->position.x);
    }

}

int main(int argc, char** argv){



    if(argc <2){
        printf("usage:\n %s config.toml\n",argv[0]);
        return 0;
    }

    bool ta_ok = ta_init(memory_pool,memory_pool + STATIC_MEMORY_SIZE,256,16,8);

    if(!ta_ok){
        printf("ta fail\n");
        return 0;
    }





    char* toml = argv[1];
//    bool is_pub = (strcmp(argv[2], "p") == 0 ) ;

    ros_handler_t handler = ros_handler_t_create();

    handler.create(&handler, toml);
    if(!handler.handler){
        handler.close(&handler);
        return 0;
    }

    // exit

    set_signal_handler(signal_handler);


    printf("dds_create_from_toml ok\n");


//    MemPool_ptr scan_pub_pool = handler.pool(&handler,"scan_pub");
//    MemPool_ptr scan_sub_pool = handler.pool(&handler,"scan_sub");

//    MemPool_ptr twist_pub_pool = handler.pool(&handler,"twist_pub");
//    MemPool_ptr twist_sub_pool = handler.pool(&handler,"twist_sub");


//    MemPool_ptr tf_pub_pool = handler.pool(&handler,"tf_pub");
//    MemPool_ptr tf_sub_pool = handler.pool(&handler,"tf_sub");

//    MemPool_ptr uint8array_pub_pool = handler.pool(&handler,"uint8array_pub");
//    MemPool_ptr uint8array_sub_pool = handler.pool(&handler,"uint8array_sub");
//    MemPool_ptr uint16array_pub_pool = handler.pool(&handler,"uint16array_pub");
//    MemPool_ptr uint16array_sub_pool = handler.pool(&handler,"uint16array_sub");



//    printf("handler.pool scan_pub_pool: %p\n",scan_pub_pool);
//    printf("handler.pool scan_sub_pool: %p\n",scan_sub_pool);
//    printf("handler.pool twist_pub_pool: %p\n",twist_pub_pool);
//    printf("handler.pool twist_sub_pool: %p\n",twist_sub_pool);
//    printf("handler.pool tf_pub_pool: %p\n",tf_pub_pool);
//    printf("handler.pool tf_sub_pool: %p\n",tf_sub_pool);

//    printf("handler.pool uint8array_pub_pool: %p\n",uint8array_pub_pool);
//    printf("handler.pool uint8array_sub_pool: %p\n",uint8array_sub_pool);
//    printf("handler.pool uint16array_pub_pool: %p\n",uint16array_pub_pool);
//    printf("handler.pool uint16array_sub_pool: %p\n",uint16array_sub_pool);

    int scan_size = 1200;
    LaserScanT_ptr send_scan_ptr = LaserScanT_alloc(scan_size);

    strcpy(send_scan_ptr->frame_id, "map");
    send_scan_ptr->range_min = 0.0f;
    send_scan_ptr->range_max = 20.0f;
    send_scan_ptr->angle_min = -1.7f;
    send_scan_ptr->angle_max = 1.7f;
    send_scan_ptr->angle_increment = (send_scan_ptr->angle_max - send_scan_ptr->angle_min) / (float)scan_size;

    float* scan_ptr_ranges = LaserScanT_get_ranges(send_scan_ptr);
    float* scan_ptr_intensities = LaserScanT_get_intensities(send_scan_ptr);

    for(int i = 0 ; i < scan_size+1;i++){
        scan_ptr_ranges[i] = i+0.5;
        scan_ptr_intensities[i] = i+0.5;
    }

    void* send_scan_buffer[] = {send_scan_ptr};


    TwistT_ptr send_twist_ptr = TwistT_alloc(0);

    send_twist_ptr->linear.x = 1.0;
    send_twist_ptr->linear.y = 2.0;
    send_twist_ptr->linear.z = 3.0;

    send_twist_ptr->angular.x = 4.0;
    send_twist_ptr->angular.y = 5.0;
    send_twist_ptr->angular.z = 6.0;

    void* send_twist_buffer[] = {send_twist_ptr};


    PoseStampedT_ptr send_tf_pose = PoseStampedT_alloc(0);
    send_tf_pose->position.x = 1.0;
    send_tf_pose->position.y = 2.0;
    send_tf_pose->position.z = 3.0;

    send_tf_pose->quaternion.w = 1.0;
    send_tf_pose->quaternion.x = 0.0;
    send_tf_pose->quaternion.y = 0.0;
    send_tf_pose->quaternion.z = 0.0;

    void* send_tf_pose_buffer[] = {send_tf_pose};


    UInt8MultiArrayT_ptr send_uint8array = UInt8MultiArrayT_alloc(10);

    UInt16MultiArrayT_ptr send_uint16array = UInt16MultiArrayT_alloc(10);
    for(int i=0;i<10;i++){
        send_uint8array->buffer[i] = i;
        send_uint16array->buffer[i] = i;
    }
    void* send_u8array_pose_buffer[] = {send_uint8array};
    void* send_u16array_pose_buffer[] = {send_uint16array};


    OccupancyGridT_ptr send_map = OccupancyGridT_alloc(100,100);
    send_map->origin.quaternion.w = 1.0;
    send_map->origin.position.x = 1.0;
    send_map->origin.position.y = 2.0;

    send_map->resolution = 0.05;
    strcpy(send_map->frame_id,"map");
    for(int i = 0 ; i < 100; i++){
        for (int j = 0 ; j < 100;j++){
            if( i == j){
                send_map->data[i*100+j] = 100;
            }else{
                send_map->data[i*100+j] = -1;
            }
        }
    }

    void* send_map_buffer[] = {send_map};


    OdometryT_ptr send_odom = OdometryT_alloc(0);

    strcpy(send_odom->frame_id,"odom");
    strcpy(send_odom->child_frame_id,"base_link");

    send_odom->twist_cov[0] = 0.01;
    send_odom->twist_cov[1*6 + 1] = 0.01;
    send_odom->twist_cov[2*6 + 2] = 0.01;
    send_odom->twist_cov[3*6 + 3] = 0.01;
    send_odom->twist_cov[4*6 + 4] = 0.01;
    send_odom->twist_cov[5*6 + 5] = 0.01;

    send_odom->pose_cov[0] = 0.01;
    send_odom->pose_cov[1*6 + 1] = 0.01;
    send_odom->pose_cov[2*6 + 2] = 0.01;
    send_odom->pose_cov[3*6 + 3] = 0.01;
    send_odom->pose_cov[4*6 + 4] = 0.01;
    send_odom->pose_cov[5*6 + 5] = 0.01;

    send_odom->pose.quaternion.w = 1.0;

    send_odom->pose.position.x = 1.0;
    send_odom->pose.position.y = 2.0;
    send_odom->pose.position.z = 3.0;

    send_odom->twist.linear.x = 0.1;
    send_odom->twist.linear.y = 0.2;
    send_odom->twist.linear.z = 0.3;
    send_odom->twist.angular.x = 0.4;
    send_odom->twist.angular.y = 0.5;
    send_odom->twist.angular.z = 0.6;

    void* send_odom_buffer[] = {send_odom};


    int cnt = 0;
    while (program_run && handler.is_ok(&handler)){
        cnt++;

        handler.write_data(&handler,"scan_pub",send_scan_buffer,1);
        handler.write_data(&handler,"twist_pub",send_twist_buffer,1);
        handler.write_data(&handler,"tf_pub",send_tf_pose_buffer,1);
        handler.write_data(&handler,"uint8array_pub",send_u8array_pose_buffer,1);
        handler.write_data(&handler,"uint16array_pub",send_u16array_pose_buffer,1);
        handler.write_data(&handler,"map_pub",send_map_buffer,1);
        handler.write_data(&handler,"odom_pub",send_odom_buffer,1);

        {
            ChannelBufferT_ptr recv_scan_buffer = handler.read_data(&handler, "scan_sub");
            if (recv_scan_buffer) {
                for (int i = 0; i < recv_scan_buffer->buffer_size; i++) {
                    LaserScanT_ptr data = (LaserScanT_ptr) recv_scan_buffer->buffer[i];
                    MLOGI("recv scan [%i], frame: %s, range :%.3f, size: %i ", i, data->frame_id, data->buffer[0],
                          data->ranges_size);

                }
            }
        }
        {
            ChannelBufferT_ptr recv_twist_buffer = handler.read_data(&handler, "twist_sub");
            if (recv_twist_buffer) {
                for (int i = 0; i < recv_twist_buffer->buffer_size; i++) {
                    TwistT_ptr data = (LaserScanT_ptr) recv_twist_buffer->buffer[i];
                    MLOGI("recv twist [%i], [%.3f, %.3f, %.3f], [%.3f, %.3f, %.3f]", i, data->linear.x, data->linear.y,
                          data->linear.z,
                          data->angular.x, data->angular.y, data->angular.z);

                }
            }
        }
        {

            send_tf_pose->quaternion.w = 1.0 * ((cnt/10)%4 == 0);
            send_tf_pose->quaternion.x = 1.0 * ((cnt/10)%4 == 1);
            send_tf_pose->quaternion.y = 1.0 * ((cnt/10)%4 == 2);
            send_tf_pose->quaternion.z = 1.0 * ((cnt/10)%4 == 3);

            MLOGI("send tf [%i], [%.3f, %.3f, %.3f], [%.3f, %.3f, %.3f, %.3f]", (cnt/10)%4, send_tf_pose->position.x,
                  send_tf_pose->position.y, send_tf_pose->position.z,
                  send_tf_pose->quaternion.w, send_tf_pose->quaternion.x, send_tf_pose->quaternion.y, send_tf_pose->quaternion.z);
            ChannelBufferT_ptr recv_tf_buffer = handler.read_data(&handler, "tf_sub");
            if (recv_tf_buffer) {
                for (int i = 0; i < recv_tf_buffer->buffer_size; i++) {
                    PoseStampedT_ptr data = (PoseStampedT_ptr) recv_tf_buffer->buffer[i];
                    MLOGI("recv tf [%i], [%.3f, %.3f, %.3f], [%.3f, %.3f, %.3f, %.3f]", i, data->position.x,
                          data->position.y, data->position.z,
                          data->quaternion.w, data->quaternion.x, data->quaternion.y, data->quaternion.z);

                }
            }
        }
        {
            ChannelBufferT_ptr recv_u8_buffer = handler.read_data(&handler,"uint8array_sub");
            if(recv_u8_buffer){
                for(int i = 0;i < recv_u8_buffer->buffer_size;i++){
                    UInt8MultiArrayT_ptr data = (UInt8MultiArrayT_ptr)recv_u8_buffer->buffer[i];
                    MLOGI("recv %s\n[","u8");
                    for(int j = 0 ; j < data->element_size;j++){
                        printf("%i, ",data->buffer[j]);
                    }
                    printf("]\n");
                }
            }
        }
        {

            ChannelBufferT_ptr recv_u16_buffer = handler.read_data(&handler,"uint16array_sub");
            if(recv_u16_buffer){
                for(int i = 0;i < recv_u16_buffer->buffer_size;i++){
                    UInt16MultiArrayT_ptr data = (UInt16MultiArrayT_ptr)recv_u16_buffer->buffer[i];
                    MLOGI("recv %s\n[","u16");
                    for(int j = 0 ; j < data->element_size;j++){
                        printf("%i, ",data->buffer[j]);
                    }
                    printf("]\n");
                }
            }
        }
        {
            int a = cnt /10;
            if(a >= 100*100){
                cnt = 0;
                a = 0;
            }
            send_map->data[a] = 100;


            ChannelBufferT_ptr recv_map_buffer = handler.read_data(&handler,"map_sub");
            if(recv_map_buffer){
                for(int i =0 ; i < recv_map_buffer->buffer_size;i++){
                    OccupancyGridT_ptr data = (OccupancyGridT_ptr )recv_map_buffer->buffer[i];
                    MLOGI("recv map origin [%.3f, %.3f, %.3f], [%.3f, %.3f, %.3f, %.3f], info: [%i, %i, %.3f], data:\n",
                          data->origin.position.x,data->origin.position.y,data->origin.position.z,
                          data->origin.quaternion.w, data->origin.quaternion.x,data->origin.quaternion.y, data->origin.quaternion.z,
                          data->width, data->height, data->resolution);

                    for(int j = 0; j< data->width;j++){
                        for(int k = 0 ; k < data->height;k++){
                            printf("%i ",data->data[j*100 + k]);
                        }
                        printf("\n");
                    }
                    printf("\n");
                }
            }
        }

        {
            ChannelBufferT_ptr recv_odom_buffer = handler.read_data(&handler,"odom_sub");

            if(recv_odom_buffer){
                for(int i = 0 ;i <recv_odom_buffer->buffer_size;i++ ){
                    OdometryT_ptr data = (OdometryT_ptr)recv_odom_buffer->buffer[i];

                    MLOGI("recv odom: frame: [%s, %s], pose: [%.3f, %.3f, %.3f], [%.3f, %.3f, %.3f, %.3f], twist [%.3f, %.3f, %.3f], [%.3f, %.3f, %.3f]\n",
                          data->frame_id, data->child_frame_id,
                          data->pose.position.x, data->pose.position.y,data->pose.position.z,
                          data->pose.quaternion.w, data->pose.quaternion.x,data->pose.quaternion.y,data->pose.quaternion.z,
                          data->twist.linear.x,data->twist.linear.y,data->twist.linear.z,data->twist.angular.x,data->twist.angular.y,data->twist.angular.z
                          );
                    printf("\npose_cov: ");
                    for(int j = 0; j < 36;j++){
                        printf("%.3f, ", data->pose_cov[j]);
                    }
                    printf("\ntwist_cov: ");
                    for(int j = 0; j < 36;j++){
                        printf("%.3f, ", data->twist_cov[j]);
                    }
                }
            }
        }


        usleep(10*1000);

    }
    handler.close(&handler);
    ta_print();
    return 0;
}