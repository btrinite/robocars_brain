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
#include <robocars_msgs/robocars_radio_channels.h>
#include <robocars_msgs/robocars_tof.h>

#include <robocars_brain_fsm.hpp>

RosInterface * ri;

class onIdle;
class onManualDriving;
class onAutonomousDriving;

class onIdle
: public RobocarsStateMachine
{
    public:
        onArm() : RobocarsStateMachine("onArm") {};

    private:

        void entry(void) override {
            RobocarsStateMachine::entry();
        };
  
        void react(ArmedEvent const & e) override { 
            RobocarsStateMachine::react(e);
            transit<onManualDriving>();
        };

        void react(TickEvent const & e) override {
            ri->maintainIdleActuators(); 
        };

};

class onManualDriving
: public RobocarsStateMachine
{
    public:
        onEnteringManual() : RobocarsStateMachine("onManualDriving") {};

    private:

        void entry(void) override {
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
        };

};

class onAutonomousDriving
: public RobocarsStateMachine
{
    public:
        onAutonomousDriving() : RobocarsStateMachine("onAutonomousDriving") {};

    protected:

        virtual void entry(void) { 
            RobocarsStateMachine::entry();
        };  

        virtual void react(TickEvent                      const & e) override { 
        };

        virtual void react(DisarmedEvent                 const & e) override { 
            RobocarsStateMachine::react(e);
            transit<onIdle>();
        };

        virtual void react(ManualDrivingEvent                     const & e) override { 
            RobocarsStateMachine::react(e);
            transit<onManualDriving>();
        };

};

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
    
    PowerTrainCmd newCmd;

    static u_int32_t last_ch5 = 0;
    static u_int32_t last_ch6 = 0;
    if (msg.ch5 != last_ch5) {
        //transition
        last_ch5=msg.ch5;
        switch (channel2Command(last_ch5)) {
            case CMD_OFF:
                send_event(DisarmedEvent());        
            break;
            case CMD_ON:
                send_event(ArmedEvent());        
            break;
        }
    }
    if (msg.ch6 != last_ch§) {
        //transition
        last_ch6=msg.ch6;
        switch (channel2Command(last_ch6)) {
            case CMD_OFF:
                send_event(ManualDrivingEvent());        
            break;
            case CMD_ON:
                send_event(AutonomousDrivingEvent());        
            break;
        }
    }
    
    newCmd.steeringCmd = msg.ch3
    newCmd.steeringCmd = msg.ch1
    send_event(PowerTraindEvent(newCmd));        
}

void MissionRosInterface::maintainIdleActuators () {

    robocars_msgs::robocars_actuator_output steeringMsg;
    robocars_msgs::robocars_actuator_output throttlingMsg;

    steeringMsg.header.stamp = ros::Time::now();
    steeringMsg.header.seq=pos_seq;
    steeringMsg.header.frame_id = "mainSteering";
    steeringMsg.pwm = 1500;

    throttlingMsg.header.stamp = ros::Time::now();
    throttlingMsg.header.seq=pos_seq;
    throttlingMsg.header.frame_id = "mainThrottling";
    throttlingMsg.pwm = 1500;   

    act_steering_pub.publish(steeringMsg);
    act_throttling_pub.publish(throttlingMsg);
}

int main(int argc, char **argv)
{
    int loopCnt=0;
    ros::init(argc, argv, "robocars_brain_fsm");

    ri = new RobocarsRosInterface;

    fsm_list::start();

    // wait for FCU connection
    ros::Rate rate(500.0);
    while(ros::ok()){
        ros::spinOnce();
        //ros::spin();
        if ((++loopCnt)%100 == 0) {
            mri->updateParam();
        }
        send_event (TickEvent());
        rate.sleep();
    }
}

