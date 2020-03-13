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
struct PowerTraindEvent             : BaseEvent { public: 
    PowerTraindEvent (const PowerTrainCmd * powerTrainCmdMsg) : powerTrainCmd(*powerTrainCmdMsg), BaseEvent("PowerTraindEvent") {};
    PowerTrainCmd powerTrainCmd; 
    };

class RobocarsStateMachine
: public tinyfsm::Fsm<RobocarsStateMachine>
{
    public:
        RobocarsStateMachine(const char * stateName) : _stateName(stateName), tinyfsm::Fsm<RobocarsStateMachine>::Fsm() { 
            ROS_INFO("RobocarsStateMachine: State created: %s", _stateName);      
        };
        const char * getStateName() const { return _stateName; };

    public:
        static float destX;
        static float destY;

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
        virtual void react(PowerTraindEvent               const & e) { };

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

/*
void state_cb(const mavros_msgs::State::ConstPtr& msg);
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
void mission_cb(const mavros_msgs::WaypointList::ConstPtr& mission_msg);
void customer_cb(const std_msgs::StringConstPtr& str);
*/

class RosInterface
{
    public :
        RosInterface() {
            updateParam();
            channels_sub = nh.subscribe<robocars_msgs::robocars_radio_channels>("radio_channels", 1, &RosInterface::channels_msg_cb, this);
            act_steering_pub = nh.advertise<robocars_msgs::robocars_actuator_output>("steering", 10);
            act_throttling_pub = nh.advertise<robocars_msgs::robocars_actuator_output>("throttling", 10);
        };

        void updateParam() {
        }

        void maintainIdleActuators();
        void RosInterface::controlActuators (PowerTrainCmd& newCmd);

    private:

        void channels_msg_cb(const robocars_msgs::robocars_radio_channels::ConstPtr& msg);

        ros::Publisher act_steering_pub;
        ros::Publisher act_throttling_pub;

        ros::NodeHandle nh;
        ros::Subscriber channels_sub;

};

