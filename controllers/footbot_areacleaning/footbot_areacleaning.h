
#ifndef FOOTBOT_AREACLEANING_H
#define FOOTBOT_AREACLEANING_H

/*
 * Include some necessary headers.
 */

#include <sstream>

/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_sensor.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
/* Definition of the foot-bot gripper actuator */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_gripper_actuator.h>
/* Definition of camera sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
/* Definition of the foot-bot motor ground sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_motor_ground_sensor.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>

#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_gripper_actuator.h>

#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <ctime>       /* time */
#include <random>
#include <cstdlib>
#include <iomanip>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotAreaCleaning : public CCI_Controller {

public:

    /* Class constructor. */
    CFootBotAreaCleaning();

    /* Class destructor. */
    virtual ~CFootBotAreaCleaning() {}

    /*
     * This function initializes the controller.
     * The 't_node' variable points to the <parameters> section in the XML
     * file in the <controllers><footbot_diffusion_controller> section.
     */
    virtual void Init(TConfigurationNode& t_node);

    /*
     * This function is called once every time step.
     * The length of the time step is set in the XML file.
     */
    virtual void ControlStep();

    /*
     * This function resets the controller to its state right after the
     * Init().
     * It is called when you press the reset button in the GUI.
     * In this example controller there is no need for resetting anything,
     * so the function could have been omitted. It's here just for
     * completeness.
     */
    virtual void Reset() {}

    /*
     * Called to cleanup what done by Init() when the experiment finishes.
     * In this example controller there is no need for clean anything up,
     * so the function could have been omitted. It's here just for
     * completeness.
     */
    virtual void Destroy();



    std::string getMessage();
    bool doYouHaveAMessage();
    void messageSend();
    void receivedMessage(std::string rMessage);

    enum State{
        EXPLORE,
        DRIVETOBOX,
        EXPERIMENT,
        BACKUP,
        FINISHED,
        STUCK
    };

private:
    void explore();
    void driveToBox();
    void experiment();
    void driveToBoxExplore();
    void backup();
    void stuck();

    bool blobInSight();
    bool colorInSight(CColor color);
    int nearestBlob(CColor color, float& dist);
    CColor chooseColorToDriveTo(bool r,bool g, bool b);
    bool newColorsInSight(bool r,bool g, bool b);

    int reward(int state, int action);
    void saUpdateNrOfTaken(int state, int action, int nrTimes);
    int numberOfTimesTaken(int state, int action);
    void updateQ(int state, int action, int reward);
    int greedyAction( int state);
    int exploreAction(int state);
    int convertColorToState(CColor cstate);
    void printQtable();
    int mgenerate(int first, int last);

    void sendMessage();
    void sendMessage(float* numoftries, int& state, int& action);
    void messageHandling();
    void clearMessage();
    /* Pointer to the differential steering actuator */
    CCI_DifferentialSteeringActuator* m_pcWheels;
    /* Pointer to the foot-bot proximity sensor */
    CCI_FootBotProximitySensor* m_pcProximity;

    /*
     * The following variables are used as parameters for the
     * algorithm. You can set their value in the <parameters> section
     * of the XML configuration file, under the
     * <controllers><footbot_diffusion_controller> section.
     */

    /* Maximum tolerance for the angle between
     * the robot heading direction and
     * the closest obstacle detected. */
    CDegrees m_cAlpha;
    /* Maximum tolerance for the proximity reading between
     * the robot and the closest obstacle.
     * The proximity reading is 0 when nothing is detected
     * and grows exponentially to 1 when the obstacle is
     * touching the robot.
     */
    Real m_fDelta;
    /* Wheel speed. */
    Real m_fWheelVelocity;
    /* Angle tolerance range to go straight.
     * It is set to [-alpha,alpha]. */
    CRange<CRadians> m_cGoStraightAngleRange;

    /* Pointer to the omnidirectional camera sensor */
    CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera;

    /* A counter used to know when to trigger each action */
    UInt64 m_unCounter;
    CCI_FootBotGripperActuator* m_pcGripper;

    CCI_RangeAndBearingSensor* m_pcRABS;
    CCI_RangeAndBearingActuator* m_pcRABA;

    CCI_DifferentialSteeringSensor *m_pcDS;

    int holdCount = 0;
    CVector2 cAccumulator;
    CRadians cAngle;
    std::ostringstream mess;

    CColor chosenColor = CColor::BLACK;


    float nrSAPairChosen[3][10] = {0} ; // number of times a action has been chosen
    float estiQ[3][10] = {0} ; //estimated state action value;

    float tempnrSAPairChosen[10] = {0};

    int collectedReward = 0;
    const std::string boxid;
    int m_Epsilon;

    std::ofstream yfile;

    int optimalchoice = 0;
    int nrOfExp = 0;

    std::vector<bool> colorMem{0,0,0};


    State state;
    int stuckRuns = 0;

    float prevBlobDist = 0;
    int stuckCounter = 0;



};

#endif
