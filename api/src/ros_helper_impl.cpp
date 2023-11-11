//
// Created by waxz on 10/19/23.
//

#include "ros_helper_impl.h"


// string
#include "absl/strings/str_format.h"
#include "absl/strings/escaping.h"
#include "absl/strings/string_view.h"
#include "absl/strings/match.h"

// log
#include "absl/log/log.h"
#include "absl/log/check.h"

// common
//#include "ros_helper_message.h"


//ROSTFReader
ROSTFReader::ROSTFReader(std::shared_ptr<tf::TransformListener> tfl) :m_tfl(tfl){}

int ROSTFReader::create_from_toml(const toml::basic_value<toml::discard_comments> &config) {
    // get parent frame and child frame
    // get wait time
    // init pool

    parent_frame = toml::find_or(config,config_key_parent_frame,"");
    child_frame = toml::find_or(config,config_key_child_frame,"");
    wait_time_s = toml::find_or(config, config_key_wait_time_s,0.0f);

    if(parent_frame.empty() || child_frame.empty()){
        LOG(INFO) << "parent_frame: [" << parent_frame<<"], child_frame: [" << child_frame << "]" ;
        return -1;
    }

    pool = MemPool_create();

    if(MemPool_set_nodes(&pool,1) == 0){
        std::cout << "ROSWriter::MemPool_set_nodes:failed, maybe ta_init is not called before" << std::endl;
        return -1;
    }
//    sample = PoseStampedT_create();
    target.frame_id_.assign(parent_frame);
    target.child_frame_id_.assign(child_frame);
    return 0;
}

int ROSTFReader::process() {
    tf::StampedTransform result;

    if(wait_time_s > 0.0f){
        m_tfl->waitForTransform(parent_frame, child_frame,ros::Time(0), ros::Duration(wait_time_s));
    }


    MemPool_reset_counter(&pool);
    try{
        ros_read_tf(m_tfl,target,&pool);
        return 1;

    }catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        return -1;
    }
    return -1;
}

ChannelBufferT_ptr ROSTFReader::read_data() {
    tf::StampedTransform result;

    if(wait_time_s > 0.0f){
        m_tfl->waitForTransform(parent_frame, child_frame,ros::Time(0), ros::Duration(wait_time_s));
    }


    MemPool_reset_counter(&pool);
    try{
        ros_read_tf(m_tfl,target,&pool);
        channel_buffer.buffer_size = pool.valid_len;
        channel_buffer.buffer = pool.nodes;
        return &channel_buffer;

    }catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        return 0;
    }
    return 0;
}

//ROSTFWriter
ROSTFWriter::ROSTFWriter(std::shared_ptr<tf::TransformBroadcaster> tfb) : m_tfb(tfb){}

int ROSTFWriter::create_from_toml(const toml::basic_value<toml::discard_comments> &config) {
    // get parent frame and child frame
    // get wait time
    // init pool

    parent_frame = toml::find_or(config,config_key_parent_frame,"");
    child_frame = toml::find_or(config,config_key_child_frame,"");
    wait_time_s = toml::find_or(config, config_key_wait_time_s,0.0f);

    if(parent_frame.empty() || child_frame.empty()){
        LOG(INFO) << "parent_frame: [" << parent_frame<<"], child_frame: [" << child_frame << "]" ;
        return -1;
    }

    pool = MemPool_create();

    if(MemPool_set_nodes(&pool,1) == 0){
        std::cout << "ROSWriter::MemPool_set_nodes:failed, maybe ta_init is not called before" << std::endl;
        return -1;
    }
//    sample = PoseStampedT_create();
    target.frame_id_.assign(parent_frame);
    target.child_frame_id_.assign(child_frame);
    return 0;
}

int ROSTFWriter::process() {

    target.stamp_ = ros::Time::now();
    ros_write_tf(m_tfb,target,&pool);
    MemPool_reset_counter(&pool);
    return 0;
}

int ROSTFWriter::write_data(void **buffer, u32 buffer_len) {
    target.stamp_ = ros::Time::now();
    ros_write_tf_data(m_tfb, target, buffer,buffer_len);
    return 0;
}

//ROSWriter
ROSWriter::ROSWriter(): m_nh(), m_nh_private("~") {

}

int ROSWriter::create_from_toml(const toml::basic_value<toml::discard_comments> &config) {

    std::strcpy(option.topic_type,toml::find_or(config,m_config_key_topic_type,"").c_str());
    std::strcpy(option.topic_name,toml::find_or(config,m_config_key_topic_name,"").c_str());
    option.queue_size = toml::find_or(config,m_config_key_qos_queue_size,1);
    option.keep_last = toml::find_or(config,m_config_key_qos_keep_last,false);

    writer =  create_writer(m_nh,&option);
    if(!writer.valid){
        std::cout << "ROSWriter::create_from_toml: " << option.topic_type << " is not defined" << std::endl;
        return -1;
    }

    pool = MemPool_create();
    if(MemPool_set_nodes(&pool,option.queue_size) == 0){
        std::cout << "ROSWriter::MemPool_set_nodes:failed, maybe ta_init is not called before" << std::endl;
        return -1;
    }

    return 0;
}

int ROSWriter::process() {

//    int ret = writer.func(writer.writer,&pool);
//    MemPool_reset_counter(&pool);
//    return ret;
    return 0;
}

int ROSWriter::process(MemPool_ptr another_pool) {

//    int ret = writer.write_data(writer.writer,another_pool);
//    MemPool_reset_counter(&pool);
//    return ret;
    return 0;

}

int ROSWriter::write_data(void **buffer, u32 buffer_len) {

    int ret = writer.write_data(writer.writer,buffer, buffer_len);
    return ret;
}

//ROSReader
ROSReader::ROSReader(): m_nh(), m_nh_private("~") {

}

int ROSReader::create_from_toml(const toml::basic_value<toml::discard_comments> &config) {

    // create option


    std::strcpy(option.topic_type,toml::find_or(config,m_config_key_topic_type,"").c_str());
    std::strcpy(option.topic_name,toml::find_or(config,m_config_key_topic_name,"").c_str());
    option.queue_size = toml::find_or(config,m_config_key_qos_queue_size,1);

    pool = MemPool_create();

    if(MemPool_set_nodes(&pool,option.queue_size) == 0){
        std::cout << "ROSWriter::MemPool_set_nodes:failed, maybe ta_init is not called before" << std::endl;
        return -1;
    }
    option.pool = &pool;

    queue = std::make_shared<ros::CallbackQueue>();

    m_nh.setCallbackQueue(queue.get());

    reader =  create_reader(m_nh,&option);
    if(!reader.valid){
        std::cout << "ROSReader::create_from_toml: " << option.topic_type << " is not defined" << std::endl;
        return -1;
    }



    return 0;

}

int ROSReader::process() {
//    std::cout << "ROSReader::process: " << __LINE__ << std::endl;
//    std::cout << "ROSReader::process: queue->isEnabled() " << queue->isEnabled() << std::endl;
//    std::cout << "ROSReader::process: queue->isEmpty() " << queue->isEmpty() << std::endl;
//    std::cout << "reader.reader.getNumPublishers(): " << reader.reader.getNumPublishers() << std::endl;
//    std::cout << "reader.reader.getTopic(): " <<reader.reader.getTopic() << std::endl;

    MemPool_reset_counter(&pool);

    queue->callAvailable(ros::WallDuration());

    return pool.valid_len;
}

ChannelBufferT_ptr ROSReader::read_data() {
    MemPool_reset_counter(&pool);

    queue->callAvailable(ros::WallDuration());

    if(pool.valid_len == 0){
        return 0;
    }else{
        channel_buffer.buffer = pool.nodes;
        channel_buffer.buffer_size = pool.valid_len;
        return &channel_buffer;
    }
    return 0;
}
int RosHandler::create_from_toml(const char *filename) {
    std::cout << "RosHandler::create_from_toml:" << filename << std::endl;
    toml::basic_value<toml::discard_comments, std::unordered_map, std::vector> toml_data;

    try{
        toml_data = toml::parse(filename);
    }catch (std::runtime_error & e){
        MLOGW("RosHandler::create_from_toml:toml::parse(%s) failed, error: %s\n",filename, e.what());
        return -1;
    }catch (toml::syntax_error& e){
        MLOGW("RosHandler::create_from_toml:toml::parse(%s) failed, error: %s\n",filename, e.what());
        return -1;
    }catch (...){
        MLOGW("RosHandler::create_from_toml:toml::parse(%s) failed, error: %s\n",filename, "Unknown");
        return -1;
    }
//    std::cout << "RosHandler::create_from_toml:toml_data:\n" << toml_data << std::endl;


    if (!toml_data.contains(m_config_key_root)){
        MLOGW("%s does not contain %s", filename, m_config_key_root);
        return -1;
    }
    auto config_root = toml::find(toml_data, m_config_key_root);
    if (!config_root.contains(m_config_key_config)){
        MLOGW("%s does not contain %s", filename, m_config_key_config);
        return -1;
    }
    auto tom_config = toml::find(config_root, m_config_key_config);

    if(!tom_config.is_table()){
        MLOGW("%s does not contain %s as table", filename, m_config_key_config);
    }

    // ros.init
    MLOGI("%s","start process args\n");

    auto toml_config_table = tom_config.as_table();
    int argc = toml_config_table.size();

    MLOGI("%s argc = %i","start process ",argc);

    std::vector<char*> arg_vec(argc);

    std::vector<std::string> config_args(argc);
    char buffer[200];
    int arg_id = 0;

    for(auto& it : toml_config_table){


        sprintf(buffer,"%s:=%s", it.first.c_str(),it.second.as_string().str.c_str());
        config_args[arg_id].assign(buffer);
        MLOGI("get arg: %s\n",buffer);
        arg_vec[arg_id] = (char*)(config_args[arg_id].data());
        arg_id++;
    }
    char** argv = arg_vec.data();


    MLOGI("%s","ros::init\n");

    ros::init(argc,argv,"demo");

    // create tfl and tfb
    bool need_tfb = false, need_tfl = false;
    m_tfl = std::make_shared<tf::TransformListener>();
    m_tfb = std::make_shared<tf::TransformBroadcaster>();

    // get all channel
    if (!config_root.contains(m_config_key_channel)){
        MLOGW("%s does not contain %s", filename, m_config_key_channel);
        return -1;
    }
    auto config_channel = toml::find(config_root, m_config_key_channel);
    std::cout << "channel:\n" << config_channel ;
    if(!tom_config.is_table()){
        MLOGW("%s does not contain %s as table", filename, m_config_key_channel);
        return -1;
    }

    std::string channel_type , topic_name, topic_type;
    int queue_size = 1;

    for(const auto& channel : config_channel.as_table()) {

        auto& channel_name = channel.first;
        auto& channel_config = channel.second;


        channel_type = toml::find_or(channel_config, m_config_key_channel_type,"");
        topic_name = toml::find_or(channel_config, m_config_key_topic_name,"");
        topic_type= toml::find_or(channel_config, m_config_key_topic_type,"");
        queue_size = toml::find_or(channel_config,m_config_key_qos_queue_size,1);


        if(channel_type.empty() || topic_name.empty() || topic_type.empty() ||
        !(std::strcmp(channel_type.c_str(),m_config_predefine_channel_type_pub) == 0 || std::strcmp(channel_type.c_str(),m_config_predefine_channel_type_sub) == 0)
        ){

            std::cout << "Detect error in channel_config:\n" << channel_config << std::endl;

            return -1;
        }
        std::cout << "\n====\nchannel_name:\n" << channel_name << "\nchannel_config:\n" << channel_config << std::endl;


        if(std::strcmp(topic_type.c_str(), m_config_predefine_topic_type_tf) ==0 && std::strcmp(topic_name.c_str(), m_config_predefine_topic_name_tf) ==0){
            if(std::strcmp(channel_type.c_str(), m_config_predefine_channel_type_pub) ==0){
                std::cout << "ROSTFWriter try create" << std::endl;

                need_tfb = true;

                auto it = channel_holder_map.emplace(channel_name, ROSTFWriter(m_tfb));

                std::cout << "ROSTFWriter ok = " << it.second << std::endl;

                int ret =  absl::get<ROSTFWriter>(it.first->second).create_from_toml(channel_config);
                std::cout << "ROSTFWriter ret = " << ret << std::endl;


            }else{
                std::cout << "ROSTFReader try create" << std::endl;

                need_tfl = true;

                auto it = channel_holder_map.emplace(channel_name, ROSTFReader(m_tfl));
                std::cout << "ROSTFReader ok = " << it.second << std::endl;

                int ret = absl::get<ROSTFReader>(it.first->second).create_from_toml(channel_config);

                std::cout << "ROSTFReader ret = " << ret << std::endl;

            }
        }else{
            if(std::strcmp(channel_type.c_str(), m_config_predefine_channel_type_pub) ==0){
                std::cout << "ROSWriter try create" << std::endl;

                auto it = channel_holder_map.emplace(channel_name, ROSWriter());
                std::cout << "ROSWriter ok = " << it.second << std::endl;

                int ret = absl::get<ROSWriter>(it.first->second).create_from_toml(channel_config);
                std::cout << "ROSWriter ret = " << ret << std::endl;

                if(ret < 0){
                    return -1;
                }

            }else{
                std::cout << "ROSReader try create" << std::endl;

                auto it = channel_holder_map.emplace(channel_name, ROSReader());
                std::cout << "ROSReader ok = " << it.second << std::endl;

                int ret = absl::get<ROSReader>(it.first->second).create_from_toml(channel_config);
                std::cout << "ROSReader ret = " << ret << std::endl;

                if(ret < 0){
                    return -1;
                }
            }


        }


    }
    std::cout << "[RosHandler]: all channel is created done" << std::endl;


    if(!need_tfl){
        m_tfl.reset();
    }
    if(!need_tfb){
        m_tfb.reset();
    }
    return 0;
}

int RosHandler::read(const char *channel_name) {
    auto it  = channel_holder_map.find(channel_name);
    if(it == channel_holder_map.end()){
        MLOGW("%s not exist\n", channel_name);
        return -1;
    }

    if( absl::holds_alternative<ROSWriter>(it->second) ){
        return -1;
    }else if( absl::holds_alternative<ROSReader>(it->second) ){
        return  absl::get<ROSReader>(it->second).process();
    }else if( absl::holds_alternative<ROSTFWriter>(it->second) ){
        return -1;
    }else if( absl::holds_alternative<ROSTFReader>(it->second) ){
        return absl::get<ROSTFReader>(it->second).process();
    }
    return -1;
}

int RosHandler::write(const char *channel_name) {

    auto it  = channel_holder_map.find(channel_name);
    if(it == channel_holder_map.end()){
        MLOGW("%s not exist\n", channel_name);
        return 0;
    }
    if( absl::holds_alternative<ROSWriter>(it->second) ){
        return  absl::get<ROSWriter>(it->second).process();
    }else if( absl::holds_alternative<ROSReader>(it->second) ){
        return 0;
    }else if( absl::holds_alternative<ROSTFWriter>(it->second) ){
        return absl::get<ROSTFWriter>(it->second).process();
    }else if( absl::holds_alternative<ROSTFReader>(it->second) ){
        return 0;
    }
    return 0;

}

int RosHandler::write(const char *channel_name, MemPool_ptr another_pool) {

    auto it  = channel_holder_map.find(channel_name);
    if(it == channel_holder_map.end()){
        MLOGW("%s not exist\n", channel_name);
        return 0;
    }
    if( absl::holds_alternative<ROSWriter>(it->second) ){
        return  absl::get<ROSWriter>(it->second).process(another_pool);
    }else if( absl::holds_alternative<ROSReader>(it->second) ){
        return 0;
    }else if( absl::holds_alternative<ROSTFWriter>(it->second) ){
        return 0;
    }else if( absl::holds_alternative<ROSTFReader>(it->second) ){
        return 0;
    }
    return 0;

}

MemPool_ptr RosHandler::get_pool(const char *channel_name) {

    auto it  = channel_holder_map.find(channel_name);
    if(it == channel_holder_map.end()){
        MLOGW("%s not exist\n", channel_name);
        return 0;
    }
    if( absl::holds_alternative<ROSWriter>(it->second) ){
        return &absl::get<ROSWriter>(it->second).pool;
    }else if( absl::holds_alternative<ROSReader>(it->second) ){
        return &absl::get<ROSReader>(it->second).pool;
    }else if( absl::holds_alternative<ROSTFWriter>(it->second) ){
        return &absl::get<ROSTFWriter>(it->second).pool;
    }else if( absl::holds_alternative<ROSTFReader>(it->second) ){
        return &absl::get<ROSTFReader>(it->second).pool;
    }
    return 0;
}

int RosHandler::stop() {
    std::cout << "RosHandler::stop" << std::endl;
    if(ros::isStarted()){
        ros::shutdown();
    }
    return 0;
}

int RosHandler::write_data(const char *channel_name, void **buffer, u32 buffer_size) {

    auto it  = channel_holder_map.find(channel_name);
    if(it == channel_holder_map.end()){
        MLOGW("%s not exist\n", channel_name);
        return 0;
    }
    if( absl::holds_alternative<ROSWriter>(it->second) ){
        return  absl::get<ROSWriter>(it->second).write_data(buffer, buffer_size);
    }else if( absl::holds_alternative<ROSReader>(it->second) ){
        return 0;
    }else if( absl::holds_alternative<ROSTFWriter>(it->second) ){
        return  absl::get<ROSTFWriter>(it->second).write_data(buffer, buffer_size);

    }else if( absl::holds_alternative<ROSTFReader>(it->second) ){
        return 0;
    }
    return 0;
}

ChannelBufferT_ptr RosHandler::read_data(const char *channel_name) {
    auto it  = channel_holder_map.find(channel_name);
    if(it == channel_holder_map.end()){
        MLOGW("%s not exist\n", channel_name);
        return 0;
    }

    if( absl::holds_alternative<ROSWriter>(it->second) ){
        return 0;
    }else if( absl::holds_alternative<ROSReader>(it->second) ){
        return  absl::get<ROSReader>(it->second).read_data();
    }else if( absl::holds_alternative<ROSTFWriter>(it->second) ){
        return 0;
    }else if( absl::holds_alternative<ROSTFReader>(it->second) ){
        return absl::get<ROSTFReader>(it->second).read_data();
    }
    return 0;
}

