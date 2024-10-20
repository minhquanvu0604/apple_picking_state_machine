#include <chrono>
#include "apple_picking_state_machine/state_machine.hpp"

//---------------------------------------------------------------------------------------------------------------------
apple_picking::StateMachine::StateMachine() : 
    sm_state_(SMState::IDLE), 
    arm_state_(ComponentState::IDLE), mapping_state_(ComponentState::IDLE)
    // progress_manager_({"Arm Module", "Mapping Process"}, {TIMEOUT, TIMEOUT})  // Initialize ProgressManager with process names and timeouts (TODO)
{
    main_thread_ = std::thread(&StateMachine::main_loop, this);

    // Service callback for starting the state machine
    ros::NodeHandle nh;
    start_srv_ = nh.advertiseService("/mvps/state_machine/start", &StateMachine::start_srv_callback, this);
    shutdown_srv_ = nh.advertiseService("/mvps/state_machine/shutdown", &StateMachine::shutdown_srv_callback, this);

    map_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("/mvps/mapping_module/map_cloud", 0, &StateMachine::map_callback, this);

    // ros::NodeHandle nh;
    // arm_state_sub_ = nh.subscribe<std_msgs::String>("mvps/arm_module/state",0, [&](const std_msgs::String::ConstPtr &_msg){
    //                                                                             arm_state_ = _msg->data;});
    // mapping_state_sub = nh.subscribe<std_msgs::String>("mvps/mapping_module/state",0, [&](const std_msgs::String::ConstPtr &_msg){
    //                                                                             mapping_state_ = _msg->data;});

    // start_srv_callback =  [&](std_srvs::Trigger::Request&, std_srvs::Trigger::Response&)->bool {
    //     sm_state_ = eState::SCANNING;
    //     return true;
    // };

    arm_client_ = nh.serviceClient<std_srvs::Trigger>("/mvps/arm_module/next_pose");
    mapping_client_ = nh.serviceClient<std_srvs::Trigger>("/mvps/mapping_module/add_frame");

    arm_client_shutdown_ = nh.serviceClient<std_srvs::Trigger>("/mvps/arm_module/shutdown");
    mapping_client_shutdown_ = nh.serviceClient<std_srvs::Trigger>("/mvps/mapping_module/shutdown");

    shutdown_.store(false);
};

void apple_picking::StateMachine::main_loop() {

    while(ros::ok()){
        if (shutdown_.load())
            break;

        std::unique_lock<std::mutex> cout_lock(cout_mtx_);
        switch(sm_state_){
        case SMState::IDLE:
            ROS_INFO("[STATE] IDLE");
            callback_idle();
            break;
        case SMState::BOTH_PROCESSING: // Both arm and mapping are processing
            ROS_INFO("[STATE] BOTH_PROCESSING");
            cout_lock.unlock();
            callback_both_processing();
            break;
        case SMState::WAIT_FOR_ARM: // Mapping is done, wait for arm to reach the next pose
            ROS_INFO("[STATE] WAIT_FOR_ARM state");
            cout_lock.unlock();
            callback_wait_for_arm();
            break;
        case SMState::WAIT_FOR_MAPPING: // Arm is done, wait for mapping to finish adding frame
            ROS_INFO("[STATE] WAIT_FOR_MAPPING state");
            cout_lock.unlock();
            callback_wait_for_mapping();
            break;
        }        
    }
    ROS_INFO("Exiting main loop");
    shutdown();
}

bool apple_picking::StateMachine::start_srv_callback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    sm_state_ = SMState::BOTH_PROCESSING;
    res.success = true;
    res.message = "State machine started!";
    return true;
}

void apple_picking::StateMachine::callback_idle() {
    arm_state_ = ComponentState::IDLE;
    mapping_state_ = ComponentState::IDLE;

    bool arm_exist = false;
    bool mapping_exist = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    while (true) {
        arm_exist = arm_client_.exists();
        mapping_exist = mapping_client_.exists();

        if (arm_exist && mapping_exist) break;

        ROS_WARN_STREAM("Services status:\tArm Module: " << arm_exist << "\tMapping Module: " << mapping_exist);
        std::this_thread::sleep_for(std::chrono::milliseconds(4000));
        if (shutdown_.load()) return;
    }
    ROS_INFO("Services are ready");

    if (sm_state_ == SMState::IDLE)
        ROS_INFO("Idling...");
    while(sm_state_ == SMState::IDLE){
        std::this_thread::sleep_for(std::chrono::milliseconds(4000));        
        if (shutdown_.load()) return;
    }

    sm_state_ = SMState::BOTH_PROCESSING;
}

void apple_picking::StateMachine::callback_both_processing() {

    // Make sure all threads finish before starting new ones
    // int t = 0;
    for (auto& thread : threads_) {
        if (thread.joinable()) thread.join();
        // t++;
    }
    // ROS_INFO_STREAM("Joined " << t << " threads");
    threads_.clear();

    arm_state_ = ComponentState::RUNNING;
    mapping_state_ = ComponentState::RUNNING;

    iter_start_time_ = std::chrono::steady_clock::now();
    // progress_manager_.update_start_time(0, iter_start_time_);
    // progress_manager_.update_start_time(1, iter_start_time_);

    if (shutdown_.load()) return;
    // std::thread arm_thread(&apple_picking::StateMachine::arm_service_thread, this);
    // std::thread mapping_thread(&apple_picking::StateMachine::mapping_service_thread, this);
    // arm_thread.detach();
    // mapping_thread.detach();

    threads_.emplace_back(&apple_picking::StateMachine::arm_service_thread, this);
    threads_.emplace_back(&apple_picking::StateMachine::mapping_service_thread, this);
    
    // Wait for either arm or mapping to finish
    std::unique_lock<std::mutex> cv_lock(cv_mtx_);
    std::unique_lock<std::mutex> cout_lock(cout_mtx_);
    while (!cv_.wait_for(cv_lock, std::chrono::milliseconds(PROGRESS_UPDATE_RATE), [this] {
        return (arm_state_ == ComponentState::DONE) || (mapping_state_ == ComponentState::DONE);
    })) {
        // Update progress bars
        // progress_manager_.update_progress(0);
        // progress_manager_.update_progress(1);

        if (shutdown_.load()){
            cout_lock.unlock();
            return;
        } 
        auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> execution_time = current_time - iter_start_time_;
        std::cout << "\r[ INFO] [BOTH_PROCESSING - Elapsed Time]: " << execution_time.count() << " seconds" << std::flush;
    }
    std::cout << std::endl;
    cout_lock.unlock();

    // Check if the state machine is done
    if (sm_state_ == SMState::IDLE){
        ROS_INFO("DEMO Done");
        return;
    }    

    // Change state based on which one finished
    if (arm_state_ == ComponentState::DONE) {
        sm_state_ = SMState::WAIT_FOR_MAPPING;
        // progress_manager_.complete_process(0); // 0 for arm module

    } else if (mapping_state_ == ComponentState::DONE) {
        sm_state_ = SMState::WAIT_FOR_ARM;
        // progress_manager_.complete_process(1); // 1 for mapping module
    }
}


void apple_picking::StateMachine::arm_service_thread() {
    // Regularly check for shutdown before making the service call
    while (!shutdown_.load()) {
        // Proceed with service call if shutdown is not triggered
        std_srvs::Trigger arm_srv;
        if (!arm_client_.call(arm_srv)) {
            std::cout << std::endl;
            ROS_ERROR("Arm service call failed");
            sm_state_ = SMState::IDLE;
            break;
        }

        // Check the response from the arm service
        if (!arm_srv.response.success) {
            sm_state_ = SMState::IDLE;
            break;
        }

        // Lock and update the component state
        {
            std::lock_guard<std::mutex> cv_lock(cv_mtx_);
            arm_state_ = ComponentState::DONE;
        }

        // Notify the state machine that the arm service is done
        cv_.notify_one();
        return; // Exit the thread as the service is done
    }

    // If shutdown is triggered during the thread execution
    if (shutdown_.load()) {
        ROS_INFO("Arm service thread exiting due to shutdown.");
        std::lock_guard<std::mutex> cv_lock(cv_mtx_);
        arm_state_ = ComponentState::DONE;
        cv_.notify_one(); // Notify the state machine that the arm service is done (due to shutdown)
    }
}


void apple_picking::StateMachine::mapping_service_thread() {
    if (shutdown_.load()) {
        ROS_INFO("Mapping service thread exiting due to shutdown.");
        std::lock_guard<std::mutex> cv_lock(cv_mtx_);
        mapping_state_ = ComponentState::DONE;
        cv_.notify_one(); // Notify the state machine that the arm service is done (due to shutdown)
        return;
    }

    std_srvs::Trigger mapping_srv;
    if (!mapping_client_.call(mapping_srv)) {
        ROS_ERROR("Mapping service call failed");
        sm_state_ = SMState::IDLE;
    }

    // The response from the mapping service is alwasy true, don't need to check
}

void apple_picking::StateMachine::map_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    std::cout << std::endl;
    // ROS_INFO("Mapping DONE - Total Elapsed Time: %f seconds", std::chrono::duration<double>(std::chrono::steady_clock::now() - iter_start_time_).count());    
    std::lock_guard<std::mutex> cv_lock(cv_mtx_);
    mapping_state_ = ComponentState::DONE;
    cv_.notify_one();
}

void apple_picking::StateMachine::callback_wait_for_arm() {
    while(arm_state_ != ComponentState::DONE){
        // progress_manager_.update_progress(0);

        auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> execution_time = current_time - iter_start_time_;
        if (shutdown_.load()) return;
        if (execution_time.count() > TIMEOUT) {
            ROS_WARN("Timeout waiting for arm to finish");
            sm_state_ = SMState::IDLE;
            return;
        }
        std::cout << "\r[ INFO] [WAIT_FOR_ARM - Total Arm Elapsed Time]: " << execution_time.count() << " seconds" << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(PROGRESS_UPDATE_RATE));
    }
    sm_state_ = SMState::BOTH_PROCESSING;
}

void apple_picking::StateMachine::callback_wait_for_mapping() {

    while(mapping_state_ != ComponentState::DONE){
        // progress_manager_.update_progress(1);
        
        auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> execution_time = current_time - iter_start_time_;
        if (shutdown_.load()) return;
        if (execution_time.count() > TIMEOUT) {
            ROS_WARN("Timeout waiting for arm to finish");
            sm_state_ = SMState::IDLE;
            return;
        }
        std::cout << "\r[ INFO] [WAIT_FOR_MAPPING - Total Mapping Elapsed Time]: " << execution_time.count() << " seconds" << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    sm_state_ = SMState::BOTH_PROCESSING;
}

bool apple_picking::StateMachine::shutdown_srv_callback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    
    ROS_INFO("Shutting down the state machine...");

    shutdown_.store(true);

    res.success = true;
    res.message = "State machine shutdown!";
    return true;
}

void apple_picking::StateMachine::shutdown() {
    try {
        // Start shutting down the components
        std_srvs::Trigger request;
        arm_client_shutdown_.call(request);
        mapping_client_shutdown_.call(request);

        // int t = 0;
        for (auto& thread : threads_) {
            if (thread.joinable()) thread.join();
            // t++;
        }
        // ROS_INFO_STREAM("Joined " << t << " threads");
        threads_.clear();
        ros::shutdown();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception during shutdown: %s", e.what());
    }
}