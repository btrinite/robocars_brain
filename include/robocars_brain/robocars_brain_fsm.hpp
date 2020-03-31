#include <tinyfsm.hpp>
#include <ros/ros.h>
#include <stdio.h>

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
        void         exit(void)  { };  /* no exit actions */

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
            updateParam();
        };

        void initPub();
        void initSub();

        void updateParam();

        void publishBrainState(uint32_t state);

    private:

        void channels_msg_cb(const robocars_msgs::robocars_radio_channels::ConstPtr& msg);

        ros::NodeHandle nh;
        ros::Publisher brain_state_pub;    
        ros::Subscriber channels_sub;

};

