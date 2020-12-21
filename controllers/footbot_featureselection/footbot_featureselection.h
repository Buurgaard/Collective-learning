
#ifndef FOOTBOT_FEATURESELECTION_H
#define FOOTBOT_FEATURESELECTION_H

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
#include <algorithm>
#include "nr3.h"
#include "svd.h"

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotFeatureSelection : public CCI_Controller {

public:

    /* Class constructor. */
    CFootBotFeatureSelection();

    /* Class destructor. */
    virtual ~CFootBotFeatureSelection() {}

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
    virtual void Reset() { }

    /*
     * Called to cleanup what done by Init() when the experiment finishes.
     * In this example controller there is no need for clean anything up,
     * so the function could have been omitted. It's here just for
     * completeness.
     */
    virtual void Destroy();


    enum State{
        EXPLORE,
        DRIVETOBOX,
        EXPERIMENT,
        BACKUP,
        FINISHED,
        STUCK
    };
    void setLookUpPointer(const double * const * lUp, unsigned int fLength);

private:
    void explore();
    void driveToBox();
    void experiment();
    void driveToBoxExplore();
    void backup();
    void stuck();

    bool blobInSight();
    int nearestBlob(CColor color, float& dist);
    int chooseCylinderToDriveTo();
    double cylinderBestEsti(const double *featureVector);


    int reward(int state, int action);
    void addData(const double *featureVector,int reward, int action);
    void updateQ(int action);
    int greedyAction( int cylinder);
    int exploreAction(int state);
    int convertColorToIndex(CColor cstate);
    void printQtable();
    int mgenerate(int first, int last);

    bool isOptimalAction(int cylinder, int action);

    void sendMessage(const double *featureVector,int reward, int action);
    void sendMessage();
    void messageHandling();
    void clearMessage();

    const float weightTable[10][3] = {
            {    1,    0,    0},
            {    0,    1,    0},
            {    0,    0,    1},
            {  0.5,  0.5,    0},
            {    0,  0.5,  0.5},
            {  0.5,    0,  0.5},
            { 0.33, 0.33, 0.33},
            {  0.5, 0.25, 0.25},
            { 0.75, 0.25,    0},
            {    0,    0,    0}
    };//rows, cols

    const double * const * lookUpPointer;
    unsigned int featureLength;

    std::vector<int> takenCylinders;

    std::ofstream term;

    std::vector<std::array<double,4>> experience[10];

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




    float estiQ[10][3] = {0} ; //estimated weights;

    float tempnrSAPairChosen[10] = {0};

    int collectedReward = 0;

    int m_Epsilon;

    std::ofstream yfile;
    std::ofstream isoptimalfile;


    int nrOfExp = 0;

    State state;
    int stuckRuns = 0;

    float prevBlobDist = 0;
    int stuckCounter = 0;

    std::vector<int> prevBlobColorIndex;
    std::vector<int> blobColorIndex;
    int chosenCylinder = -1;
};

#endif
