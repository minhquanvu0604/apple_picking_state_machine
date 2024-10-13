#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include "apple_picking_state_machine/state_machine.hpp"

//---------------------------------------------------------------------------------------------------------------------
apple_picking::StateMachine::StateMachine() : 
    sm_state_(SMState::IDLE), 
    arm_state_(ComponentState::IDLE), mapping_state_(ComponentState::IDLE) 
{
    main_thread_ = std::thread(&StateMachine::main_loop, this);

    // Service callback for starting the state machine
    ros::NodeHandle nh;
    start_srv_ = nh.advertiseService("mvps/state_machine/start", &StateMachine::start_srv_callback, this);


    // ros::NodeHandle nh;
    // arm_state_sub_ = nh.subscribe<std_msgs::String>("mvps/arm_module/state",0, [&](const std_msgs::String::ConstPtr &_msg){
    //                                                                             arm_state_ = _msg->data;});
    // mapping_state_sub = nh.subscribe<std_msgs::String>("mvps/mapping_module/state",0, [&](const std_msgs::String::ConstPtr &_msg){
    //                                                                             mapping_state_ = _msg->data;});

    // start_srv_callback =  [&](std_srvs::Trigger::Request&, std_srvs::Trigger::Response&)->bool {
    //     sm_state_ = eState::SCANNING;
    //     return true;
    // };
};

void apple_picking::StateMachine::main_loop() {

    while(ros::ok()){

        ROS_INFO_STREAM("NEW ITERATION, sm_state_: " << static_cast<int>(sm_state_.load()));

        switch(sm_state_){
        case SMState::IDLE:
            ROS_INFO("IDLE state");
            callback_idle();
            break;
        case SMState::BOTH_PROCESSING: // Both arm and mapping are processing
            ROS_INFO("BOTH_PROCESSING state");
            callback_both_processing();
            ROS_INFO("BOTH_PROCESSING state DONE");
            break;
        case SMState::WAIT_FOR_ARM: // Mapping is done, wait for arm to reach the next pose
            ROS_INFO("WAIT_FOR_ARM state");
            callback_wait_for_arm();
            break;
        case SMState::WAIT_FOR_MAPPING: // Arm is done, wait for mapping to finish adding frame
            ROS_INFO("WAIT_FOR_MAPPING state");
            callback_wait_for_mapping();
            break;
        }
    }
}

void apple_picking::StateMachine::callback_idle() {
    arm_state_ = ComponentState::IDLE;
    mapping_state_ = ComponentState::IDLE;

    while(sm_state_ == SMState::IDLE){
        ROS_INFO("Idling...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    sm_state_ = SMState::BOTH_PROCESSING;
}

void apple_picking::StateMachine::callback_both_processing() {
    arm_state_ = ComponentState::RUNNING;
    mapping_state_ = ComponentState::RUNNING;

    std::thread arm_thread(&apple_picking::StateMachine::arm_service_thread, this);
    std::thread mapping_thread(&apple_picking::StateMachine::mapping_service_thread, this);
    arm_thread.detach();
    mapping_thread.detach();
    
    std::unique_lock<std::mutex> lock(mtx_);
    cv_.wait(lock, [this] {
        return (arm_state_ ==  ComponentState::DONE) || (mapping_state_ == ComponentState::DONE);
    });

    ROS_INFO("PASSING Both processing");
    ROS_INFO_STREAM("sm_state_: " << static_cast<int>(sm_state_.load()));

    // Check if the state machine is done
    if (sm_state_ == SMState::IDLE){
        ROS_INFO("DEMO Done");
        return;
    }    

    ROS_INFO("PASSING Both processing 2");


    // Change state based on which one finished
    if (arm_state_ == ComponentState::DONE) {
        sm_state_ = SMState::WAIT_FOR_MAPPING;
    } else if (mapping_state_ == ComponentState::DONE) {
        sm_state_ = SMState::WAIT_FOR_ARM;
    }
}

void apple_picking::StateMachine::arm_service_thread() {
    ros::NodeHandle nh;
    ros::ServiceClient arm_client = nh.serviceClient<std_srvs::Trigger>("mvps/arm_module/next_pose");

    std_srvs::Trigger arm_srv;
    ROS_INFO("Calling arm service");
    if (arm_client.call(arm_srv)) {
        ROS_INFO("Arm service call success");
    } else {
        ROS_INFO("Arm service call failed");
        sm_state_ = SMState::IDLE;
    }
    ROS_INFO("Arm service done");

    std::lock_guard<std::mutex> lock(mtx_);
    arm_state_ = ComponentState::DONE;
    cv_.notify_all(); // Notify that the arm service is done
}

void apple_picking::StateMachine::mapping_service_thread() {
    ros::NodeHandle nh;
    ros::ServiceClient mapping_client = nh.serviceClient<std_srvs::Trigger>("mvps/mapping_module/add_frame");

    std_srvs::Trigger mapping_srv;
    ROS_INFO("Calling mapping service");
    if (mapping_client.call(mapping_srv)) {
        ROS_INFO("Mapping service call success");
    } else {
        ROS_INFO("Mapping service call failed");
        sm_state_ = SMState::IDLE;
    }
    ROS_INFO("Mapping service done");

    std::lock_guard<std::mutex> lock(mtx_);
    mapping_state_ = ComponentState::DONE;
    cv_.notify_all(); // Notify that the mapping service is done
}

void apple_picking::StateMachine::callback_wait_for_arm() {
    while(arm_state_ != ComponentState::DONE){
        ROS_INFO("Waiting for arm...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    sm_state_ = SMState::BOTH_PROCESSING;
}

void apple_picking::StateMachine::callback_wait_for_mapping() {
    while(mapping_state_ != ComponentState::DONE){
        ROS_INFO("Waiting for mapping...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    sm_state_ = SMState::BOTH_PROCESSING;
}

bool apple_picking::StateMachine::start_srv_callback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    sm_state_ = SMState::BOTH_PROCESSING;
    res.success = true;
    res.message = "State machine started!";
    return true;
}