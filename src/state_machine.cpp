#include <ros/package.h>

#include <chrono>
#include <ctime>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include <filesystem>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "apple_picking_state_machine/state_machine.hpp"

//---------------------------------------------------------------------------------------------------------------------
apple_picking::StateMachine::StateMachine() : 
    sm_state_(SMState::IDLE), 
    arm_state_(ComponentState::IDLE), mapping_state_(ComponentState::IDLE),
    arm_total_time_(std::chrono::duration<double>::zero()), mapping_total_time_(std::chrono::duration<double>::zero()), 
    shutdown_(false)
    // progress_manager_({"Arm Module", "Mapping Process"}, {TIMEOUT, TIMEOUT})  // Initialize ProgressManager with process names and timeouts (TODO)
{
    // shutdown_.store(false);
    main_thread_ = std::thread(&StateMachine::main_loop, this);

    // Service callback for starting the state machine
    ros::NodeHandle nh;
    start_srv_ = nh.advertiseService("/mvps/state_machine/start", &StateMachine::start_srv_callback, this);
    shutdown_srv_ = nh.advertiseService("/mvps/state_machine/shutdown", &StateMachine::shutdown_srv_callback, this);

    map_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("/mvps/mapping_module/map_cloud", 0, &StateMachine::map_callback, this);

    arm_client_ = nh.serviceClient<std_srvs::Trigger>("/mvps/arm_module/next_pose");
    mapping_client_ = nh.serviceClient<std_srvs::Trigger>("/mvps/mapping_module/add_frame");

    arm_client_shutdown_ = nh.serviceClient<std_srvs::Trigger>("/mvps/arm_module/shutdown");
    mapping_client_shutdown_ = nh.serviceClient<std_srvs::Trigger>("/mvps/mapping_module/shutdown");

    // Subscribe to the map_cloud and save it to a file
    std::string yaml_file = ros::package::getPath("apple_picking_state_machine") + "/config/components_coordination_config.yaml";
    YAML::Node config = YAML::LoadFile(yaml_file);
    bool save_pointcloud = config["map_cloud"]["save"].as<bool>(true);  // Default to true if not found

    if (save_pointcloud) {
        std::string save_dir = config["map_cloud"]["save_dir"].as<std::string>();    
        
        // Create the directory if it doesn't exist
        std::filesystem::path dirPath(save_dir);
        if (!std::filesystem::exists(dirPath)) 
            if (std::filesystem::create_directories(dirPath))
                ROS_INFO_STREAM("Directory created: " << save_dir);
            else
                ROS_ERROR("Failed to create directory: %s", save_dir.c_str());
        
        auto now = std::chrono::system_clock::now(); // Get the current time
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);

        std::stringstream datetime;
        datetime << std::put_time(std::localtime(&now_c), "%Y_%m_%d_%H_%M_%S"); // Format the time as YYYY_MM_DD_HH_MM_SS

        pointcloud_filename_ = save_dir + "/map_cloud_" + datetime.str() + ".pcd";

        ROS_INFO("Saving the mapped cloud to file %s whenever it is subscribed", pointcloud_filename_.c_str());
        pointcloud_save_sub_ = nh.subscribe("/mvps/mapping_module/map_cloud", 10, &StateMachine::map_cloud_callback, this);
    }
};

void apple_picking::StateMachine::main_loop() {
    while(ros::ok()){
        if (shutdown_.load()){
            ROS_INFO("Shutdown signal received");
            break;
        }

        std::unique_lock<std::mutex> cout_lock(cout_mtx_);
        switch(sm_state_){
        case SMState::IDLE:
            ROS_INFO("[STATE] IDLE");
            callback_idle();
            break;
        case SMState::BOTH_PROCESSING: // Both arm and mapping are processing
            // Start the timer for the first iteration        
            if (iteration_counter_ == 1)
                system_start_time_ = std::chrono::steady_clock::now();    
            std::cout << "\n-------------------- ITERATION " << iteration_counter_ << " --------------------" << std::endl;
            iteration_counter_++;

            ROS_INFO("[STATE] Both Arm and Mapping Module Processing");
            cout_lock.unlock();
            callback_both_processing();
            break;
        case SMState::WAIT_FOR_ARM: // Mapping is done, wait for arm to reach the next pose
            ROS_INFO("[STATE] Wait for Arm Module");
            cout_lock.unlock();
            callback_wait_for_arm();
            break;
        case SMState::WAIT_FOR_MAPPING: // Arm is done, wait for mapping to finish adding frame
            ROS_INFO("[STATE] Wait for Mapping Module");
            cout_lock.unlock();
            callback_wait_for_mapping();
            break;
        }        
    }
    ROS_INFO("Exiting main loop");

    summary();
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
    }
    cout_lock.unlock();

    // Either Arm or Mapping is done
    // Update the elapsed time
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> execution_time = current_time - iter_start_time_;

    // Check if the system has finished
    if (sm_state_ == SMState::IDLE){
        ROS_INFO("DEMO Done");
        arm_total_time_ += execution_time;
        mapping_total_time_ += execution_time;
        shutdown_.store(true);
        return;
    }    

    // Move to the next state based on which component is done
    if (arm_state_ == ComponentState::DONE){
        ROS_INFO_STREAM("[Arm Module] Elapsed Time: " << execution_time.count() << " seconds");
        arm_total_time_ += execution_time;
        sm_state_ = SMState::WAIT_FOR_MAPPING;
    }
    else if (mapping_state_ == ComponentState::DONE){
        ROS_INFO_STREAM("[Mapping Module] Elapsed Time: " << execution_time.count() << " seconds");
        mapping_total_time_ += execution_time;
        sm_state_ = SMState::WAIT_FOR_ARM;
    } else {
        // For logic debugging
        ROS_ERROR("Both arm and mapping are not done");
    }
}


void apple_picking::StateMachine::arm_service_thread() {
    // Regularly check for shutdown before making the service call
    while (!shutdown_.load()) {
        // Proceed with service call if shutdown is not triggered
        std_srvs::Trigger arm_srv;
        if (!arm_client_.call(arm_srv)) {
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
    std::lock_guard<std::mutex> cv_lock(cv_mtx_);
    mapping_state_ = ComponentState::DONE;
    cv_.notify_one();
}

void apple_picking::StateMachine::callback_wait_for_arm() {
    std::chrono::duration<double> execution_time = std::chrono::duration<double>::zero();
    while(arm_state_ != ComponentState::DONE){
        // progress_manager_.update_progress(0);
        if (shutdown_.load()) return;

        auto current_time = std::chrono::steady_clock::now();
        execution_time = current_time - iter_start_time_;
        if (execution_time.count() > TIMEOUT) {
            ROS_WARN("Timeout waiting for arm to finish");
            sm_state_ = SMState::IDLE;
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(PROGRESS_UPDATE_RATE));
    }
    ROS_INFO_STREAM("[Arm Module] Elapsed Time: " << execution_time.count() << " seconds");
    arm_total_time_ += execution_time;
    sm_state_ = SMState::BOTH_PROCESSING;
}

void apple_picking::StateMachine::callback_wait_for_mapping() {
    std::chrono::duration<double> execution_time = std::chrono::duration<double>::zero();
    while(mapping_state_ != ComponentState::DONE){
        // progress_manager_.update_progress(1);
        if (shutdown_.load()) return;

        auto current_time = std::chrono::steady_clock::now();
        execution_time = current_time - iter_start_time_;
        if (execution_time.count() > TIMEOUT) {
            ROS_WARN("Timeout waiting for arm to finish");
            sm_state_ = SMState::IDLE;
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    ROS_INFO_STREAM("[Mapping Module] Elapsed Time: " << execution_time.count() << " seconds");
    mapping_total_time_ += execution_time;
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

void apple_picking::StateMachine::summary() {
    std::cout << "\n-------------------- SUMMARY --------------------" << std::endl;
    std::cout << "After " << iteration_counter_ << " iterations:" << std::endl;

    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> total_execution_time = current_time - system_start_time_;
    ROS_INFO_STREAM("System Execution Time: " << total_execution_time.count() << " seconds");
    ROS_INFO_STREAM("Arm Module Total Execution Time: " << arm_total_time_.count() << " seconds");
    ROS_INFO_STREAM("Mapping Module Total Execution Time: " << mapping_total_time_.count() << " seconds");
}

void apple_picking::StateMachine::map_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // Convert ROS PointCloud2 message to PCL point cloud with color
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
    pcl::fromROSMsg(*msg, pcl_cloud);

    // Verify each point has color data (optional: assign default color if missing)
    for (auto& point : pcl_cloud.points) {
        if (point.r == 0 && point.g == 0 && point.b == 0) {
            point.r = 255;  // Set default color (e.g., white or any desired color) if none is present
            point.g = 255;
            point.b = 255;
        }
    }

    // Save the PCL point cloud in binary format to keep color information
    if (pcl::io::savePCDFileBinary(pointcloud_filename_, pcl_cloud) == -1) {
        ROS_ERROR("Failed to save colored point cloud to file: %s", pointcloud_filename_.c_str());
    } else {
        ROS_INFO("Colored point cloud successfully saved to %s", pointcloud_filename_.c_str());
    }
}
