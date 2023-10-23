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

configure_file(${CMAKE_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/include/ros/common.h.in ${CMAKE_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/include/ros/common.h)

# Output test results to config.h
#configure_file(./libros/libros/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)
configure_file(${CMAKE_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/config.h.in ${CMAKE_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/config.h)

configure_file(${CMAKE_BINARY_DIR}/console_bridge/console_bridge_export.h ${CMAKE_SOURCE_DIR}/libros/console_bridge-master/include/console_bridge/console_bridge_export.h COPYONLY)

### boost
#https://stackoverflow.com/questions/33242956/cmake-passing-lists-on-command-line
build_external_project(Boost lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/boost-1.82.0.tar.xz -DCMAKE_BUILD_TYPE=Release '-DBOOST_INCLUDE_LIBRARIES=thread;chrono;filesystem;system;date_time;regex' -DBUILD_SHARED_LIBS=OFF )

#find_package(boost_timer REQUIRED)
#find_package(boost_filesystem REQUIRED)
set(BOOST_REQUIRED_MODULES thread chrono filesystem system date_time regex)

find_package(Boost 1.82.0 REQUIRED COMPONENTS ${BOOST_REQUIRED_MODULES})

set(BUILD_SHARED_LIBS OFF)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libros/console_bridge-master console_bridge)

set(LIBROS_SOURCE_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/actionlib-noetic-devel/actionlib/src/connection_monitor.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/actionlib-noetic-devel/actionlib/src/goal_id_generator.cpp


         ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/utilities/xmlrpcpp/src/XmlRpcClient.cpp
         ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/utilities/xmlrpcpp/src/XmlRpcDispatch.cpp
         ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/utilities/xmlrpcpp/src/XmlRpcServer.cpp
         ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/utilities/xmlrpcpp/src/XmlRpcServerConnection.cpp
         ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/utilities/xmlrpcpp/src/XmlRpcServerMethod.cpp
         ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/utilities/xmlrpcpp/src/XmlRpcSocket.cpp
         ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/utilities/xmlrpcpp/src/XmlRpcSource.cpp
         ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/utilities/xmlrpcpp/src/XmlRpcUtil.cpp
         ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/utilities/xmlrpcpp/src/XmlRpcValue.cpp
         ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/utilities/xmlrpcpp/libb64/src/cdecode.c
         ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/utilities/xmlrpcpp/libb64/src/cencode.c

         ${CMAKE_CURRENT_SOURCE_DIR}/libros/rosconsole-noetic-devel/src/rosconsole/rosconsole_backend.cpp
         ${CMAKE_CURRENT_SOURCE_DIR}/libros/rosconsole-noetic-devel/src/rosconsole/rosconsole.cpp
         ${CMAKE_CURRENT_SOURCE_DIR}/libros/rosconsole-noetic-devel/src/rosconsole/impl/rosconsole_print.cpp

        ${CMAKE_CURRENT_SOURCE_DIR}/libros/geometry2-noetic-devel/tf2/src/cache.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/geometry2-noetic-devel/tf2/src/buffer_core.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/geometry2-noetic-devel/tf2/src/static_cache.cpp

        ${CMAKE_CURRENT_SOURCE_DIR}/libros/geometry2-noetic-devel/tf2_ros/src/buffer.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/geometry2-noetic-devel/tf2_ros/src/transform_listener.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/geometry2-noetic-devel/tf2_ros/src/buffer_client.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/geometry2-noetic-devel/tf2_ros/src/buffer_server.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/geometry2-noetic-devel/tf2_ros/src/transform_broadcaster.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/geometry2-noetic-devel/tf2_ros/src/static_transform_broadcaster.cpp

        ${CMAKE_CURRENT_SOURCE_DIR}/libros/roscpp_core-noetic-devel/cpp_common/src/debug.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/roscpp_core-noetic-devel/cpp_common/src/header.cpp

        ${CMAKE_CURRENT_SOURCE_DIR}/libros/roscpp_core-noetic-devel/rostime/src/rate.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/roscpp_core-noetic-devel/rostime/src/duration.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/roscpp_core-noetic-devel/rostime/src/time.cpp

        ${CMAKE_CURRENT_SOURCE_DIR}/libros/roscpp_core-noetic-devel/roscpp_serialization/src/serialization.cpp


        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/master.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/network.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/subscriber.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/common.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/publisher_link.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/service_publication.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/connection.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/single_subscriber_publisher.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/param.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/service_server.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/wall_timer.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/xmlrpc_manager.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/publisher.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/timer.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/io.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/names.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/topic.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/topic_manager.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/poll_manager.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/publication.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/statistics.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/intraprocess_subscriber_link.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/intraprocess_publisher_link.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/callback_queue.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/service_server_link.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/service_client.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/node_handle.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/connection_manager.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/file_log.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/transport/transport.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/transport/transport_udp.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/transport/transport_tcp.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/subscriber_link.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/service_client_link.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/transport_publisher_link.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/transport_subscriber_link.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/service_manager.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/rosout_appender.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/init.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/subscription.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/subscription_queue.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/spinner.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/internal_timer_manager.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/message_deserializer.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/poll_set.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/service.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/this_node.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/steady_timer.cpp

)



set(LIBROS_INCLUDE_DIR
        ${Boost_INCLUDE_DIRS}
#        ${CMAKE_CURRENT_BINARY_DIR}
#        ${CMAKE_CURRENT_BINARY_DIR}/console_bridge

        ${CMAKE_CURRENT_SOURCE_DIR}/libros/include

        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/include

        ${CMAKE_CURRENT_SOURCE_DIR}/libros/actionlib-noetic-devel/actionlib/include
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/utilities/xmlrpcpp/libb64/include
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/utilities/xmlrpcpp/include
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/rosconsole-noetic-devel/include
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/geometry2-noetic-devel/tf2/include
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/geometry2-noetic-devel/tf2_ros/include

        ${CMAKE_CURRENT_SOURCE_DIR}/libros/roscpp_core-noetic-devel/roscpp_traits/include
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/roscpp_core-noetic-devel/cpp_common/include
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/roscpp_core-noetic-devel/rostime/include
        ${CMAKE_CURRENT_SOURCE_DIR}/libros/roscpp_core-noetic-devel/roscpp_serialization/include

)

add_library(libros STATIC ${LIBROS_SOURCE_FILES})
target_include_directories(libros PUBLIC ${LIBROS_INCLUDE_DIR} )
target_link_libraries(libros PUBLIC ${Boost_LIBRARIES} console_bridge)



#build_external_project_cmake_git(roscpp_core lib https://github.com/ros/roscpp_core.git 0.7.2)
add_executable(libros_test main.cpp)
target_link_libraries(libros_test PUBLIC libros )


##### install
INSTALL(DIRECTORY ${LIBROS_INCLUDE_DIR} DESTINATION ${CMAKE_INSTALL_PREFIX})
INSTALL(TARGETS libros DESTINATION lib)