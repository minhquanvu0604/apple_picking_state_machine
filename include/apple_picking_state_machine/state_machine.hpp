#include <atomic>
#include <thread>
#include <condition_variable>
#include <mutex>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

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

        bool start_srv_callback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);


    private:    // Members
        std::atomic<SMState> sm_state_;

        // std::thread mSpinThread;
        // ros::ServiceServer mStateService;

        ros::Subscriber arm_state_sub_, mapping_state_sub_;
        ComponentState arm_state_, mapping_state_;
        
        std::condition_variable cv_;
        std::thread main_thread_;
        std::mutex mtx_;

        ros::ServiceServer start_srv_;


        // std::thread state_pub_thread_;
        // ros::Publisher state_pub_;
    };
}
