#include <atomic>
#include <thread>
#include <condition_variable>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>

// #include "apple_picking_state_machine/progress_manager.hpp"


namespace apple_picking {

    enum class SMState {
        IDLE, 
        BOTH_PROCESSING, 
        WAIT_FOR_ARM,
        WAIT_FOR_MAPPING 
    };

    enum class ComponentState {
        IDLE,
        RUNNING,
        DONE
    };

    class StateMachine{
    public:
        StateMachine();

    private:
        void main_loop();

        void callback_idle();
        void callback_both_processing();
        void callback_wait_for_arm();
        void callback_wait_for_mapping();

        void arm_service_thread();
        void mapping_service_thread();
        void map_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);


        bool start_srv_callback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
        bool shutdown_srv_callback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
        void shutdown();
        

    private:    // Members
        std::atomic<SMState> sm_state_;

        std::vector<std::thread> threads_;

        ros::ServiceServer start_srv_, shutdown_srv_;        
        std::atomic<bool> shutdown_;

        // std::thread mSpinThread;
        // ros::ServiceServer mStateService;
        ros::ServiceClient arm_client_, mapping_client_;
        ros::ServiceClient arm_client_shutdown_, mapping_client_shutdown_;

        // ros::Subscriber arm_state_sub_, mapping_state_sub_;
        ComponentState arm_state_, mapping_state_;

        ros::Subscriber map_sub_;
        
        std::condition_variable cv_;
        std::thread main_thread_;
        std::mutex cv_mtx_, time_mtx_, cout_mtx_;

        // std::thread state_pub_thread_;
        // ros::Publisher state_pub_;

        std::chrono::steady_clock::time_point iter_start_time_;

        const int TIMEOUT = 300; // seconds
        const int PROGRESS_UPDATE_RATE = 100; // milliseconds

        // ProgressManager progress_manager_;
    };
}
