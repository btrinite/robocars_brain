/**
 * @file robocars_brain_fsm.hpp
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

#include <std_msgs/Int16MultiArray.h>

#include <robocars_msgs/robocars_actuator_output.h>
#include <robocars_msgs/robocars_debug.h>
#include <robocars_msgs/robocars_led_status.h>
#include <robocars_msgs/robocars_radio_channels.h>
#include <robocars_msgs/robocars_tof.h>

typedef struct {
    u_int32_t steeringCmd;
    u_int32_t throttlingCmd;
} PowerTrainCmd;

struct BaseEvent : tinyfsm::Event
{
    public:
        BaseEvent(const char * evtName) : _evtName(evtName) {};
        const char * getEvtName() const { return _evtName; };
    private:
        const char *  _evtName;
};

struct TickEvent                    : BaseEvent { public: TickEvent() : BaseEvent("TickEvent") {}; };
struct ArmedEvent                   : BaseEvent { public: ArmedEvent() : BaseEvent("ArmedEvent") {}; };
struct DisarmedEvent                : BaseEvent { public: DisarmedEvent() : BaseEvent("DisarmedEvent") {}; };
struct ManualDrivingEvent           : BaseEvent { public: ManualDrivingEvent() : BaseEvent("ManualDrivingEvent") {}; };
struct AutonomousDrivingEvent       : BaseEvent { public: AutonomousDrivingEvent() : BaseEvent("AutonomousDrivingEvent") {}; };
struct NominalADEvent               : BaseEvent { public: NominalADEvent() : BaseEvent("NominalADEven") {}; };

class RobocarsStateMachine
: public tinyfsm::Fsm<RobocarsStateMachine>
{
    public:
        RobocarsStateMachine(const char * stateName) : _stateName(stateName), tinyfsm::Fsm<RobocarsStateMachine>::Fsm() { 
            ROS_INFO("RobocarsStateMachine: State created: %s", _stateName);      
        };
        const char * getStateName() const { return _stateName; };

    public:
        /* default reaction for unhandled events */
        void react(BaseEvent const & ev) { 
            ROS_INFO("state %s: unexpected event %s reveived", getStateName(), ev.getEvtName());      
        };

        virtual void react(TickEvent                      const & e) { /*logEvent(e);*/ };
        virtual void react(ArmedEvent                     const & e) { logEvent(e); };
        virtual void react(DisarmedEvent                  const & e) { logEvent(e); };
        virtual void react(ManualDrivingEvent             const & e) { logEvent(e); };
        virtual void react(AutonomousDrivingEvent         const & e) { logEvent(e); };
        virtual void react(NominalADEvent                 const & e) { logEvent(e); };

        virtual void entry(void) { 
            ROS_INFO("State %s: entering", getStateName()); 
        };  
        virtual void exit(void)  { };  /* no exit actions */

    private:
        const char *  _stateName ="NoName";
        void logEvent(BaseEvent const & e) {
            ROS_INFO("State %s: event %s", getStateName(), e.getEvtName());
        }
};

typedef tinyfsm::FsmList<RobocarsStateMachine> fsm_list;

template<typename E>
void send_event(E const & event)
{
  fsm_list::template dispatch<E>(event);
}

class RosInterface
{
    public :
        RosInterface() {
            initParam();
            updateParam();
        };

        void initPub();
        void initSub();

        void initParam();
        void updateParam();

        void getIPAddress();
        void publishBrainState(uint32_t state);
        void publishDebug(std::string const& msg);

    private:

        void channels_msg_cb(const std_msgs::Int16MultiArray::ConstPtr& msg);
        void rc_driving_msg_cb(const std_msgs::Int16::ConstPtr& msg);
        void rc_autopilot_msg_cb(const std_msgs::Int16::ConstPtr& msg);

        ros::NodeHandle nh;
        ros::Publisher brain_state_pub;    
        ros::Publisher debug_pub;    
        ros::Subscriber channels_sub;
        ros::Subscriber remote_control_driving;
        ros::Subscriber remote_control_autopilot;

};

