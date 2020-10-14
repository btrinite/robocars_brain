/**
 * @file robocars_brain_fsm.cpp
 * @brief Main car FSM, hold car global state.
 * 
 * Copyright (c) 2020 Benoit TRINITE
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * Topic subscribed : 
 *  - /radio_channels : to get command from radio controller
 * 
 * Topic published :
 *  - /robocars_brain_state : to broadcast car state to any other ROS node
 *  - /robocars_debug : to send debug message on ESP32 companion 
 * 
 * Parameters :
 *  - mainIPInterface : the name of the main IP interface for which current IP address is reported in debug msg
 */

#include <tinyfsm.hpp>
#include <ros/ros.h>
#include <stdio.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>
 
#include <robocars_msgs/robocars_actuator_output.h>
#include <robocars_msgs/robocars_debug.h>
#include <robocars_msgs/robocars_led_status.h>
#include <robocars_msgs/robocars_brain_state.h>
#include <robocars_msgs/robocars_radio_channels.h>

#include <robocars_brain_fsm.hpp>

#define LOOPHZ  1 /*config update and debug msg rate for IP address reporting*/

RosInterface * ri;
std::string myIP;
static std::string mainIPInterface;


class onIdle;
class onManualDriving;
class onAutonomousDriving;

class onIdle
: public RobocarsStateMachine
{
    public:
        onIdle() : RobocarsStateMachine("onArm") {};

    private:
        uint32_t __tick_count;

        void entry(void) override {
            ri->publishBrainState(robocars_msgs::robocars_brain_state::BRAIN_STATE_IDLE);
            RobocarsStateMachine::entry();
        };
  
        void react(ArmedEvent const & e) override { 
            RobocarsStateMachine::react(e);
            transit<onManualDriving>();
        };

        void react(TickEvent const & e) override {
            __tick_count++;
            ri->publishBrainState(robocars_msgs::robocars_brain_state::BRAIN_STATE_IDLE);
            if ((__tick_count%LOOPHZ)==0) {
                //Update param each second
                ri->updateParam(); 
                ri->publishDebug(myIP);
            }
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

// logic to convert radio channel to state 
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
    u_int32_t ch5_cmd = channel2Command(msg->channels[4]);
    u_int32_t ch6_cmd = channel2Command(msg->channels[5]);
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

void RosInterface::initParam() {
    if (!nh.hasParam("mainIPInterface")) {
        nh.setParam ("mainIPInterface", "lo");       
    }
}
void RosInterface::updateParam() {
    nh.getParam("mainIPInterface", mainIPInterface);
}

void RosInterface::initPub () {
    brain_state_pub = nh.advertise<robocars_msgs::robocars_brain_state>("robocars_brain_state", 1);
    debug_pub = nh.advertise<robocars_msgs::robocars_debug>("robocars_debug", 1);
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

void RosInterface::getIPAddress()
{
    int fd;
    struct ifreq ifr;
    char ip_address[15];

    /*AF_INET - to define network interface IPv4*/
    /*Creating soket for it.*/
    fd = socket(AF_INET, SOCK_DGRAM, 0);
     
    /*AF_INET - to define IPv4 Address type.*/
    ifr.ifr_addr.sa_family = AF_INET;
     
    /*eth0 - define the ifr_name - port name
    where network attached.*/
    memcpy(ifr.ifr_name, mainIPInterface.c_str(), IFNAMSIZ-1);
     
    /*Accessing network interface information by
    passing address using ioctl.*/
    ioctl(fd, SIOCGIFADDR, &ifr);
    /*closing fd*/
    close(fd);
     
    /*Extract IP Address*/
    strcpy(ip_address,inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));
    myIP = std::string(ip_address); 
}

void RosInterface::publishDebug (std::string const& msg) {
    robocars_msgs::robocars_debug debugMsg;

    debugMsg.header.stamp = ros::Time::now();
    debugMsg.header.seq=1;
    debugMsg.header.frame_id = "debug";
    debugMsg.msg = msg;

    debug_pub.publish(debugMsg);
}

int main(int argc, char **argv)
{
    int loopCnt=0;
    ros::init(argc, argv, "robocars_brain_fsm");

    ri = new RosInterface;

    ri->initPub();
    fsm_list::start();
    ri->initSub();
    ri->getIPAddress();
    ROS_INFO("Brain: Starting");

    // wait for FCU connection
    ros::Rate rate(LOOPHZ);
    while(ros::ok()){
        ros::spinOnce();
        //ros::spin();
        if ((++loopCnt)%(1000/LOOPHZ) == 0) {
            ri->updateParam();
        }
        send_event (TickEvent());
        rate.sleep();
    }
}

