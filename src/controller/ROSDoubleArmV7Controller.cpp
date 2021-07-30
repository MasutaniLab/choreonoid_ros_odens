/**
   @author Yasuhiro Masutani
*/

#include <cnoid/SimpleController>
#include <ros/node_handle.h>
#include <std_msgs/Float32MultiArray.h>
#include <mutex>

#include <cnoid/EigenUtil>

using namespace std;
using namespace cnoid;

class ROSDoubleArmV7Controller : public cnoid::SimpleController
{
    ros::NodeHandle node;

    ros::Subscriber anglesTargetSubscriber;
    std_msgs::Float32MultiArray anglesTargetLast;
    std::mutex anglesTargetMutex;
    std_msgs::Float32MultiArray anglesTarget;
    bool anglesTargetIsNew;

    ros::Subscriber velocitiesSubscriber;
    std_msgs::Float32MultiArray velocitiesLast;
    std::mutex velocitiesMutex;
    std_msgs::Float32MultiArray velocities;
    bool velocitiesIsNew;

    Body* body;
    double dt;

    int mainActuationMode;

    enum TrackType { NO_TRACKS = 0, CONTINOUS_TRACKS, PSEUDO_TRACKS };
    int trackType;
    Link* trackL;
    Link* trackR;
    double trackgain;

    vector<int> armJointIdMap;
    vector<Link*> armJoints;
    vector<double> q_ref;
    vector<double> q_prev;
    vector<double> pgain;
    vector<double> dgain;


public:
    ROSDoubleArmV7Controller();
    virtual bool configure(SimpleControllerConfig* config) override;
    virtual bool initialize(SimpleControllerIO* io) override;
    void anglesTargetCallback(const std_msgs::Float32MultiArray& msg);
    void velocitiesCallback(const std_msgs::Float32MultiArray& msg);
    bool initContinuousTracks(SimpleControllerIO* io);
    bool initPseudoContinuousTracks(SimpleControllerIO* io);
    void initArms(SimpleControllerIO* io);
    void initPDGain();
    void initJoystickKeyBind();
    virtual bool control() override;
    void controlTracks();
    void setTargetArmPositions();
    void controlArms();
    void controlArmsWithTorque();
    void controlArmsWithVelocity();
    void controlArmsWithPosition();

    Link* link(const char* name) { return body->link(name); }
};


ROSDoubleArmV7Controller::ROSDoubleArmV7Controller()
{
    mainActuationMode = Link::JointEffort;
    trackType = NO_TRACKS;
}

bool ROSDoubleArmV7Controller::configure(SimpleControllerConfig* config)
{
    //config->sigChanged().connect();
    return true;
}

bool ROSDoubleArmV7Controller::initialize(SimpleControllerIO* io)
{
    body = io->body();
    dt = io->timeStep();

    io->os() << "The actuation mode of " << io->controllerName() << " is ";
    string option = io->optionString();
    if(option == "velocity"){
        mainActuationMode = Link::JointVelocity;
        io->os() << "JOINT_VELOCITY";
    } else if(option  == "position"){
        mainActuationMode = Link::JointDisplacement;
        io->os() << "JOINT_DISPLACEMENT";
    } else {
        mainActuationMode = Link::JointEffort;
        io->os() << "JOINT_EFFORT";
    }
    io->os() << "." << endl;

    initContinuousTracks(io) || initPseudoContinuousTracks(io);
    
    initArms(io);
    initPDGain();

    anglesTargetSubscriber = node.subscribe("qt", 1, &ROSDoubleArmV7Controller::anglesTargetCallback, this);
    velocitiesSubscriber = node.subscribe("vel", 1, &ROSDoubleArmV7Controller::velocitiesCallback, this);

    anglesTargetIsNew = false;
    velocitiesIsNew = false;

    return true;
}

void ROSDoubleArmV7Controller::anglesTargetCallback(const std_msgs::Float32MultiArray& msg)
{
    std::lock_guard<std::mutex> lock(anglesTargetMutex);
    anglesTargetLast = msg;
    anglesTargetIsNew = true;
}

void ROSDoubleArmV7Controller::velocitiesCallback(const std_msgs::Float32MultiArray& msg)
{
    std::lock_guard<std::mutex> lock(velocitiesMutex);
    velocitiesLast = msg;
    velocitiesIsNew = true;;
}

bool ROSDoubleArmV7Controller::initContinuousTracks(SimpleControllerIO* io)
{
    trackL = link("WHEEL_L0");
    trackR = link("WHEEL_R0");

    if(!trackL || !trackR){
        return false;
    }
    
    if(mainActuationMode == Link::JointEffort){
        trackL->setActuationMode(Link::JointEffort);
        trackR->setActuationMode(Link::JointEffort);
    } else {
        trackL->setActuationMode(Link::JointVelocity);
        trackR->setActuationMode(Link::JointVelocity);
    }
    
    io->enableOutput(trackL);
    io->enableOutput(trackR);

    trackType = CONTINOUS_TRACKS;
    
    io->os() << "Continuous tracks of " << body->name() << " are detected." << endl;
    
    return true;
}


bool ROSDoubleArmV7Controller::initPseudoContinuousTracks(SimpleControllerIO* io)
{
    trackL = link("TRACK_L");
    trackR = link("TRACK_R");

    if(!trackL || !trackR){
        return false;
    }

    if(trackL->actuationMode() == Link::JointVelocity && trackR->actuationMode() == Link::JointVelocity){
        io->enableOutput(trackL);
        io->enableOutput(trackR);
        trackType = PSEUDO_TRACKS;
        io->os() << "Pseudo continuous tracks of " << body->name() << " are detected." << endl;
    }

    return (trackType == PSEUDO_TRACKS);
}


void ROSDoubleArmV7Controller::initArms(SimpleControllerIO* io)
{
    armJointIdMap.clear();
    armJoints.clear();
    q_ref.clear();
    for(auto joint : body->joints()){
        if(joint->jointId() >= 0 && (joint->isRevoluteJoint() || joint->isPrismaticJoint())){
            joint->setActuationMode(mainActuationMode);
            io->enableIO(joint);
            armJointIdMap.push_back(armJoints.size());
            armJoints.push_back(joint);
            q_ref.push_back(joint->q());
        } else {
            armJointIdMap.push_back(-1);
        }
    }
    q_prev = q_ref;
}

void ROSDoubleArmV7Controller::initPDGain()
{
    // Tracks
    if(trackType == CONTINOUS_TRACKS){
        if(mainActuationMode == Link::JointEffort){
            trackgain = 2000.0;
        } else {
            trackgain = 2.0;
        }
    } else if(trackType == PSEUDO_TRACKS){
        trackgain = 1.0;
    }

    // Arm
    if(mainActuationMode == Link::JointEffort){
        pgain = {
        /* MFRAME */ 200000, /* BLOCK */ 150000, /* BOOM */ 150000, /* ARM  */ 100000,
        /* PITCH  */  30000, /* ROLL  */  20000, /* TIP1 */    500, /* TIP2 */    500,
        /* UFRAME */ 150000, /* SWING */  50000, /* BOOM */ 100000, /* ARM  */  80000,
        /* ELBOW */   30000, /* YAW   */  20000, /* HAND */    500, /* ROD  */  50000};
        dgain = {
        /* MFRAME */ 20000, /* BLOCK */ 15000, /* BOOM */ 10000, /* ARM  */ 5000,
        /* PITCH  */   500, /* ROLL  */   500, /* TIP1 */    50, /* TIP2 */   50,
        /* UFRAME */ 15000, /* SWING */  1000, /* BOOM */  3000, /* ARM  */ 2000,
        /* ELBOW */    500, /* YAW   */   500, /* HAND */    20, /* ROD  */ 5000};

    } else if(mainActuationMode == Link::JointVelocity){
        pgain = {
        /* MFRAME */ 100, /* BLOCK */ 100, /* BOOM */ 100, /* ARM  */ 100,
        /* PITCH  */  50, /* ROLL  */  50, /* TIP1 */   5, /* TIP2 */   5,
        /* UFRAME */ 100, /* SWING */ 100, /* BOOM */ 100, /* ARM  */ 100,
        /* ELBOW */   50, /* YAW   */  20, /* HAND */  20, /* ROD  */  50};
    }
}


bool ROSDoubleArmV7Controller::control()
{
    {
        std::lock_guard<std::mutex> lock(velocitiesMutex);
        if (velocitiesIsNew) {
            velocities = velocitiesLast;
            velocitiesIsNew = false;
            if (velocities.data.size() >= 2) {
                controlTracks();
            }
        }
    }

    {
        std::lock_guard<std::mutex> lock(anglesTargetMutex);
        if (anglesTargetIsNew) {
            anglesTarget = anglesTargetLast;
            anglesTargetIsNew = false;
            if (anglesTarget.data.size() >= armJoints.size()) {
                setTargetArmPositions();
            }
        }
    }
  
    controlArms();

    return true;
}

void ROSDoubleArmV7Controller::controlTracks()
{
    trackL->u() = 0.0;
    trackL->dq_target() = 0.0;
    trackR->u() = 0.0;
    trackR->dq_target() = 0.0;
    if(trackType == CONTINOUS_TRACKS 
        && mainActuationMode == Link::ActuationMode::JOINT_EFFORT){
        trackL->u() = trackgain * velocities.data[0];
        trackR->u() = trackgain * velocities.data[1];
    } else {
        trackL->dq_target() = trackgain * velocities.data[0];
        trackR->dq_target() = trackgain * velocities.data[1];
    }
}


void ROSDoubleArmV7Controller::setTargetArmPositions()
{
    static const double maxerror = radian(3.0);
    for(size_t i=0; i < armJoints.size(); ++i){
        auto joint = armJoints[i];
        auto& q = q_ref[i];
        q = anglesTarget.data[i];
#if 0
        auto q_current = joint->q();
        auto q_lower = std::max(q_current - maxerror, joint->q_lower());
        auto q_upper = std::min(q_current + maxerror, joint->q_upper());
#else
        auto q_lower = joint->q_lower();
        auto q_upper = joint->q_upper();
#endif
        if(q < q_lower){
            q = q_lower;
        } else if(q > q_upper){
            q = q_upper;
        }
    }
}


void ROSDoubleArmV7Controller::controlArms()
{
    switch(mainActuationMode){
    case Link::JointDisplacement:
        controlArmsWithPosition();
        break;
    case Link::JointVelocity:
        controlArmsWithVelocity();
        break;
    case Link::JointEffort:
        controlArmsWithTorque();
        break;
    default:
        break;
    }
}


void ROSDoubleArmV7Controller::controlArmsWithPosition()
{
    for(size_t i=0; i < armJoints.size(); ++i){
        armJoints[i]->q_target() = q_ref[i];
    }
}


void ROSDoubleArmV7Controller::controlArmsWithVelocity()
{
    for(size_t i=0; i < armJoints.size(); ++i){
        auto joint = armJoints[i];
        auto q_current = joint->q();
        joint->dq_target() = pgain[i] * (q_ref[i] - q_current);
    }
}


void ROSDoubleArmV7Controller::controlArmsWithTorque()
{
    for(size_t i=0; i < armJoints.size(); ++i){
        auto joint = armJoints[i];
        auto q_current = joint->q();
        auto dq_current = (q_current - q_prev[i]) / dt;
        joint->u() = pgain[i] * (q_ref[i] - q_current) + dgain[i] * (0.0 - dq_current);
        q_prev[i] = q_current;
    }
}


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(ROSDoubleArmV7Controller)
