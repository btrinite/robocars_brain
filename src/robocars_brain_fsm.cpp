/**
 * @file offb_raw_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 source $src_path/Tools/setup_gazebo.bash ${src_path} ${build_path}

 gzserver --verbose ${src_path}/Tools/sitl_gazebo/worlds/${model}.world &
 */
#include <tinyfsm.hpp>
#include <ros/ros.h>
#include <stdio.h>

#include <robocars_msgs/robocars_actuator_output.h>
#include <robocars_msgs/robocars_debug.h>
#include <robocars_msgs/robocars_led_status.h>
#include <robocars_msgs/robocars_brain_state.h>
#include <robocars_msgs/robocars_radio_channels.h>

#include <robocars_brain_fsm.hpp>

RosInterface * ri;

class onIdle;
class onManualDriving;
class onAutonomousDriving;

class onIdle
: public RobocarsStateMachine
{
    public:
        onIdle() : RobocarsStateMachine("onArm") {};

    private:

        void entry(void) override {
            ri->publishBrainState(robocars_msgs::robocars_brain_state::BRAIN_STATE_IDLE);
            RobocarsStateMachine::entry();
        };
  
        void react(ArmedEvent const & e) override { 
            RobocarsStateMachine::react(e);
            transit<onManualDriving>();
        };

        void react(TickEvent const & e) override {
            ri->publishBrainState(robocars_msgs::robocars_brain_state::BRAIN_STATE_IDLE);
        };

};

class onManualDriving
: public RobocarsStateMachine
{
    public:
        onManualDriving() : RobocarsStateMachine("onManualDriving") {};

    private:

        void entry(void) override {
            ri->publishBrainState(robocars_msgs::robocars_brain_state::BRAIN_STATE_MANUAL_DRIVING);
            RobocarsStateMachine::entry();
        };

        void react (AutonomousDrivingEvent const & e) override {
            RobocarsStateMachine::react(e);
            transit<onAutonomousDriving>();
        }

        void react(DisarmedEvent const & e) override { 
            RobocarsStateMachine::react(e);
            transit<onIdle>();
        };

        void react (TickEvent const & e) override { 
            ri->publishBrainState(robocars_msgs::robocars_brain_state::BRAIN_STATE_MANUAL_DRIVING);
        };

};

class onAutonomousDriving
: public RobocarsStateMachine
{
    public:
        onAutonomousDriving() : RobocarsStateMachine("onAutonomousDriving") {};

    protected:

        virtual void entry(void) { 
            ri->publishBrainState(robocars_msgs::robocars_brain_state::BRAIN_STATE_AUTONOMOUS_DRIVING);
            RobocarsStateMachine::entry();
        };  

        virtual void react(DisarmedEvent                 const & e) override { 
            RobocarsStateMachine::react(e);
            transit<onIdle>();
        };

        virtual void react(ManualDrivingEvent                     const & e) override { 
            RobocarsStateMachine::react(e);
            transit<onManualDriving>();
        };

        virtual void react(TickEvent                      const & e) override { 
            ri->publishBrainState(robocars_msgs::robocars_brain_state::BRAIN_STATE_AUTONOMOUS_DRIVING);
        };


};

FSM_INITIAL_STATE(RobocarsStateMachine, onIdle)

#define CMD_OFF 0
#define CMD_AMBIGIOUS 1
#define CMD_ON 2

u_int8_t channel2Command (u_int32_t channelValue) {
    if ((channelValue)<1400) {
        return CMD_OFF;
    }
    if ((channelValue)>1600) {
        return CMD_ON;
    }
    return CMD_AMBIGIOUS;
}

void RosInterface::channels_msg_cb(const robocars_msgs::robocars_radio_channels::ConstPtr& msg){
    
    static u_int32_t last_ch5_cmd = 0;
    static u_int32_t last_ch6_cmd = 0;
    u_int32_t ch5_cmd = channel2Command(msg->ch5);
    u_int32_t ch6_cmd = channel2Command(msg->ch6);
    if (ch5_cmd != last_ch5_cmd) {
        //transition
        last_ch5_cmd=ch5_cmd;
        switch (ch5_cmd) {
            case CMD_OFF:
                send_event(DisarmedEvent());        
            break;
            case CMD_ON:
                send_event(ArmedEvent());        
            break;
        }
    }
    if (ch6_cmd != last_ch6_cmd) {
        //transition
        last_ch6_cmd=ch6_cmd;
        switch (ch6_cmd) {
            case CMD_OFF:
                send_event(ManualDrivingEvent());        
            break;
            case CMD_ON:
                send_event(AutonomousDrivingEvent());        
            break;
        }
    }    
}

void RosInterface::updateParam() {
}

void RosInterface::initPub () {
    brain_state_pub = nh.advertise<robocars_msgs::robocars_brain_state>("robocars_brain_state", 10);
}

void RosInterface::initSub () {
    channels_sub = nh.subscribe<robocars_msgs::robocars_radio_channels>("radio_channels", 1, &RosInterface::channels_msg_cb, this);
}

void RosInterface::publishBrainState (uint32_t state) {

    robocars_msgs::robocars_brain_state newState;

    newState.header.stamp = ros::Time::now();
    newState.header.seq=1;
    newState.header.frame_id = "brainState";
    newState.state = state;

    brain_state_pub.publish(newState);
}

int main(int argc, char **argv)
{
    int loopCnt=0;
    ros::init(argc, argv, "robocars_brain_fsm");

    ri = new RosInterface;

    ri->initPub();
    fsm_list::start();
    ri->initSub();

    ROS_INFO("Brain: Starting");

    // wait for FCU connection
    ros::Rate rate(10.0);
    while(ros::ok()){
        ros::spinOnce();
        //ros::spin();
        if ((++loopCnt)%100 == 0) {
            ri->updateParam();
        }
        send_event (TickEvent());
        rate.sleep();
    }
}

