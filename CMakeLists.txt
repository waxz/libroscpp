cmake_minimum_required(VERSION 3.16)
project(libroscpp)
include(cmake/common.cmake)
include(cmake/set_rpath.cmake)
include(cmake/super_build.cmake)
include(cmake/find_opencv_helper.cmake)
set(CMAKE_CXX_STANDARD  14)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/sanitizers")





###############
## roscpp_VERSION_MAJOR
###########
list(GET roscpp_VERSION_LIST 0 roscpp_VERSION_MAJOR)
list(GET roscpp_VERSION_LIST 1 roscpp_VERSION_MINOR)
list(GET roscpp_VERSION_LIST 2 roscpp_VERSION_PATCH)

#configure_file(./libros/include/ros/common.h.in ./libros/include/ros/common.h)

configure_file(./libros/ros_comm-noetic-devel/clients/roscpp/include/ros/common.h.in ./libros/ros_comm-noetic-devel/clients/roscpp/include/ros/common.h)

# Output test results to config.h
#configure_file(./libros/libros/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)
configure_file(./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/config.h.in ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/config.h)

configure_file(${CMAKE_BINARY_DIR}/console_bridge/console_bridge_export.h ./libros/console_bridge-master/include/console_bridge/console_bridge_export.h COPYONLY)
configure_file(${CMAKE_BINARY_DIR}/console_bridge/console_bridge_export.h ${CMAKE_BINARY_DIR}/console_bridge_export.h  COPYONLY)

### boost
#https://stackoverflow.com/questions/33242956/cmake-passing-lists-on-command-line
build_external_project(Boost lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/boost-1.82.0.tar.xz -DCMAKE_BUILD_TYPE=Release '-DBOOST_INCLUDE_LIBRARIES=thread;chrono;filesystem;system;date_time;regex' -DBUILD_SHARED_LIBS=OFF )

#find_package(boost_timer REQUIRED)
#find_package(boost_filesystem REQUIRED)
set(BOOST_REQUIRED_MODULES thread chrono filesystem system date_time regex)

find_package(Boost 1.82.0 REQUIRED COMPONENTS ${BOOST_REQUIRED_MODULES})


add_subdirectory(./libros/console_bridge-master console_bridge)

set(LIBROS_SOURCE_FILES
        ./libros/actionlib-noetic-devel/actionlib/src/connection_monitor.cpp
        ./libros/actionlib-noetic-devel/actionlib/src/goal_id_generator.cpp


         ./libros/ros_comm-noetic-devel/utilities/xmlrpcpp/src/XmlRpcClient.cpp
         ./libros/ros_comm-noetic-devel/utilities/xmlrpcpp/src/XmlRpcDispatch.cpp
         ./libros/ros_comm-noetic-devel/utilities/xmlrpcpp/src/XmlRpcServer.cpp
         ./libros/ros_comm-noetic-devel/utilities/xmlrpcpp/src/XmlRpcServerConnection.cpp
         ./libros/ros_comm-noetic-devel/utilities/xmlrpcpp/src/XmlRpcServerMethod.cpp
         ./libros/ros_comm-noetic-devel/utilities/xmlrpcpp/src/XmlRpcSocket.cpp
         ./libros/ros_comm-noetic-devel/utilities/xmlrpcpp/src/XmlRpcSource.cpp
         ./libros/ros_comm-noetic-devel/utilities/xmlrpcpp/src/XmlRpcUtil.cpp
         ./libros/ros_comm-noetic-devel/utilities/xmlrpcpp/src/XmlRpcValue.cpp
         ./libros/ros_comm-noetic-devel/utilities/xmlrpcpp/libb64/src/cdecode.c
         ./libros/ros_comm-noetic-devel/utilities/xmlrpcpp/libb64/src/cencode.c

         ./libros/rosconsole-noetic-devel/src/rosconsole/rosconsole_backend.cpp
         ./libros/rosconsole-noetic-devel/src/rosconsole/rosconsole.cpp
         ./libros/rosconsole-noetic-devel/src/rosconsole/impl/rosconsole_print.cpp

        ./libros/geometry2-noetic-devel/tf2/src/cache.cpp
        ./libros/geometry2-noetic-devel/tf2/src/buffer_core.cpp
        ./libros/geometry2-noetic-devel/tf2/src/static_cache.cpp

        ./libros/geometry2-noetic-devel/tf2_ros/src/buffer.cpp
        ./libros/geometry2-noetic-devel/tf2_ros/src/transform_listener.cpp
        ./libros/geometry2-noetic-devel/tf2_ros/src/buffer_client.cpp
        ./libros/geometry2-noetic-devel/tf2_ros/src/buffer_server.cpp
        ./libros/geometry2-noetic-devel/tf2_ros/src/transform_broadcaster.cpp
        ./libros/geometry2-noetic-devel/tf2_ros/src/static_transform_broadcaster.cpp

        ./libros/roscpp_core-noetic-devel/cpp_common/src/debug.cpp
        ./libros/roscpp_core-noetic-devel/cpp_common/src/header.cpp

        ./libros/roscpp_core-noetic-devel/rostime/src/rate.cpp
        ./libros/roscpp_core-noetic-devel/rostime/src/duration.cpp
        ./libros/roscpp_core-noetic-devel/rostime/src/time.cpp

        ./libros/roscpp_core-noetic-devel/roscpp_serialization/src/serialization.cpp


        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/master.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/network.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/subscriber.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/common.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/publisher_link.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/service_publication.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/connection.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/single_subscriber_publisher.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/param.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/service_server.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/wall_timer.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/xmlrpc_manager.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/publisher.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/timer.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/io.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/names.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/topic.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/topic_manager.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/poll_manager.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/publication.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/statistics.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/intraprocess_subscriber_link.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/intraprocess_publisher_link.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/callback_queue.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/service_server_link.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/service_client.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/node_handle.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/connection_manager.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/file_log.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/transport/transport.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/transport/transport_udp.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/transport/transport_tcp.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/subscriber_link.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/service_client_link.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/transport_publisher_link.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/transport_subscriber_link.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/service_manager.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/rosout_appender.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/init.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/subscription.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/subscription_queue.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/spinner.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/internal_timer_manager.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/message_deserializer.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/poll_set.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/service.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/this_node.cpp
        ./libros/ros_comm-noetic-devel/clients/roscpp/src/libros/steady_timer.cpp

)



set(LIBROS_INCLUDE_DIR
        ${Boost_INCLUDE_DIRS}
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}/console_bridge

        ./libros/include

        ./libros/ros_comm-noetic-devel/clients/roscpp/include

        ./libros/actionlib-noetic-devel/actionlib/include
        ./libros/ros_comm-noetic-devel/utilities/xmlrpcpp/libb64/include
        ./libros/ros_comm-noetic-devel/utilities/xmlrpcpp/include
        ./libros/rosconsole-noetic-devel/include
        ./libros/geometry2-noetic-devel/tf2/include
        ./libros/geometry2-noetic-devel/tf2_ros/include

        ./libros/roscpp_core-noetic-devel/roscpp_traits/include
        ./libros/roscpp_core-noetic-devel/cpp_common/include
        ./libros/roscpp_core-noetic-devel/rostime/include
        ./libros/roscpp_core-noetic-devel/roscpp_serialization/include

)

add_library(libros STATIC ${LIBROS_SOURCE_FILES})
target_include_directories(libros PUBLIC ${LIBROS_INCLUDE_DIR} )
target_link_libraries(libros PUBLIC ${Boost_LIBRARIES} console_bridge)



#build_external_project_cmake_git(roscpp_core lib https://github.com/ros/roscpp_core.git 0.7.2)
add_executable(libros_test main.cpp)
target_link_libraries(libros_test PUBLIC libros )