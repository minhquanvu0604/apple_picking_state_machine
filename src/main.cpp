#include "apple_picking_state_machine/state_machine.hpp"

int main(int _argc, char **_argv){

    

    ros::init(_argc, _argv, "state_machine");
    apple_picking::StateMachine machine;

    // Endless spin
    ros::spin();
}
