#ifndef feature_selection_LOOP_FUNCTIONS_H
#define feature_selection_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/simulator/media/led_medium.h>
/* box entity def */
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include <algorithm>

using namespace argos;

class CFeatureSelectionLoopFunctions : public CLoopFunctions {

public:

    CFeatureSelectionLoopFunctions();
    virtual ~CFeatureSelectionLoopFunctions() {}

    virtual void Init(TConfigurationNode& t_tree);
    virtual void Reset();
    virtual void Destroy();
    virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
    void RemoveEntity ( const std::string& str_entity_id);
    virtual void PreStep();
    virtual void PostStep();
    bool isCloseEnough(CFootBotEntity& sender ,CFootBotEntity& receiver);


private:
    int mgenerate(int first, int last);

    CFloorEntity* m_pcFloor;
    CRandom::CRNG *m_pcRNG;

    std::vector<std::string> currentGrab;
    std::vector<std::string> previousGrab;
    int canRemove = 0;

    std::string m_strOutput;
    std::ofstream m_cOutput;

    int totalReward = 0;

    double** lookUp;
    std::ofstream term;


};

#endif
