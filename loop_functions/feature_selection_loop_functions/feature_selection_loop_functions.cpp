#include "feature_selection_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/footbot_featureselection/footbot_featureselection.h>
#include <argos3/core/utility/configuration/argos_exception.h>

/****************************************/
/****************************************/

CFeatureSelectionLoopFunctions::CFeatureSelectionLoopFunctions() :
        m_pcFloor(NULL),
        m_pcRNG(NULL){
}

/****************************************/
/****************************************/

void CFeatureSelectionLoopFunctions::Init(TConfigurationNode& t_node) {

    try {
            /* Get a pointer to the floor entity */
        m_pcFloor = &GetSpace().GetFloorEntity();
        m_pcRNG = CRandom::CreateRNG("argos");



    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
    }

    term = std::ofstream("/dev/tty", std::ios_base::out);

    std::cout<<"hello"<<std::endl;

    int featureLength = 3;
    std::vector<CRange<int>> features;
    // the loop up table could be filled with numbers between 0 and one, and it would give less code, but i like
    // that it simulates concrete data being filled into the look up table. It requires some extra work to add a new
    // feature to the feature vector. You can use less features by adjusting the variable featureLength down.
    CRange<int> weight(100,5000); // in gram
    CRange<int> height(50,500);   // in mm
    CRange<int> diameter(50,500);   // in mm

    features.push_back(weight);
    features.push_back(height);
    features.push_back(diameter);



    CSpace::TMapPerType& m_cCylinder = GetSpace().GetEntitiesByType("cylinder");
    lookUp = new double* [m_cCylinder.size()];

    for ( auto cyl : m_cCylinder) {
        std::string cId = cyl.first;
        cId.std::string::erase(0,1);
        int cylId = std::stoi(cId);
        lookUp[cylId] = new double[featureLength];
        for (int i = 0; i < featureLength; ++i) {
            CRange<int> newRange = features[i];
            int entityFeature = mgenerate(newRange.GetMin(), newRange.GetMax());
            lookUp[cylId][i] = newRange.NormalizeValue(entityFeature);
        }


        std::cout<<cylId<<std::endl;
        CCylinderEntity &cylEnti = *any_cast<CCylinderEntity *>(cyl.second);
        CColor cylColor(255, (cylId>>8)&0xff, cylId&0xff);
        cylEnti.GetLEDEquippedEntity().SetLEDColor(0, cylColor);

    }

    CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    for(auto fbot:m_cFootbots){
        CFootBotEntity& sFootBot = *any_cast<CFootBotEntity *>(fbot.second);
        auto& sController = dynamic_cast<CFootBotFeatureSelection&>(sFootBot.GetControllableEntity().GetController());
        sController.setLookUpPointer(lookUp,featureLength);
    }

}

/****************************************/
/****************************************/

void CFeatureSelectionLoopFunctions::Reset() {

    int featureLength = 3;
    std::vector<CRange<int>> features;
    // the loop up table could be filled with numbers between 0 and one, and it would give less code, but i like
    // that it simulates concrete data being filled into the look up table. It requires some extra work to add a new
    // feature to the feature vector. You can use less features by adjusting the variable featureLength down.
    CRange<int> weight(100,5000); // in gram
    CRange<int> height(50,500);   // in mm
    CRange<int> diameter(50,500);   // in mm

    features.push_back(weight);
    features.push_back(height);
    features.push_back(diameter);



    CSpace::TMapPerType& m_cCylinder = GetSpace().GetEntitiesByType("cylinder");

    for (int i = 0; i < m_cCylinder.size(); ++i) {
        delete[] lookUp[i];
    }
    delete[] lookUp;

    lookUp = new double* [m_cCylinder.size()];

    for ( auto cyl : m_cCylinder) {
        std::string cId = cyl.first;
        cId.std::string::erase(0,1);
        int cylId = std::stoi(cId);
        lookUp[cylId] = new double[featureLength];
        for (int i = 0; i < featureLength; ++i) {
            CRange<int> newRange = features[i];
            int entityFeature = mgenerate(newRange.GetMin(), newRange.GetMax());
            lookUp[cylId][i] = newRange.NormalizeValue(entityFeature);
        }


        std::cout<<cylId<<std::endl;
        CCylinderEntity &cylEnti = *any_cast<CCylinderEntity *>(cyl.second);
        CColor cylColor(255, (cylId>>8)&0xff, cylId&0xff);
        cylEnti.GetLEDEquippedEntity().SetLEDColor(0, cylColor);

    }

    CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");
    for(auto fbot:m_cFootbots){
        CFootBotEntity& sFootBot = *any_cast<CFootBotEntity *>(fbot.second);
        auto& sController = dynamic_cast<CFootBotFeatureSelection&>(sFootBot.GetControllableEntity().GetController());
        sController.setLookUpPointer(lookUp,featureLength);
    }


}

/****************************************/
/****************************************/

void CFeatureSelectionLoopFunctions::Destroy() {

}

/****************************************/
/****************************************/

CColor CFeatureSelectionLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
    if(c_position_on_plane.GetX() < -15.0f || c_position_on_plane.GetX() > 15.0f || c_position_on_plane.GetY() < -15.0f || c_position_on_plane.GetY() > 15.0f) {
        return CColor::GRAY40;      // nest area
    }
    return CColor::WHITE;           // foraging area
}

/****************************************/
/****************************************/
void CFeatureSelectionLoopFunctions::PreStep() {
    CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");




    for(CSpace::TMapPerType::iterator it = m_cFootbots.begin(); it != m_cFootbots.end(); ++it) {
        /* Get handle to foot-bot entity and controller */
        CFootBotEntity& sFootBot = *any_cast<CFootBotEntity *>(it->second);
        CFootBotFeatureSelection& sController = dynamic_cast<CFootBotFeatureSelection&>(sFootBot.GetControllableEntity().GetController());
        CGripperEquippedEntity m_pcGripperEquippedEntity = sFootBot.GetComponent<CGripperEquippedEntity>("gripper");

        if(m_pcGripperEquippedEntity.IsGripping()){
            //std::cout<< m_pcGripperEquippedEntity.GetGrippedEntity().GetIndex() << std::endl;
            //std::cout<< m_pcGripperEquippedEntity.GetGrippedEntity().GetParent().GetId() << std::endl;
            currentGrab.push_back(m_pcGripperEquippedEntity.GetGrippedEntity().GetParent().GetId());
        }


    }

}

void CFeatureSelectionLoopFunctions::PostStep() {
    for(auto& id : currentGrab){
        std::vector<std::string>::iterator position = std::find(previousGrab.begin(), previousGrab.end(), id);
        if(position != previousGrab.end()){
            previousGrab.erase(position);  //erase elemtent that is no longer grapped
        }
    }

    for(auto& id: previousGrab){
        if(id[0] != 'f') {
            //CLoopFunctions::RemoveEntity(id); //remove boxes from sim
            totalReward++;
            CVector3 pos ((rand()%1480 -730)/100.0f,(rand()%1480 -730)/100.0f,0.0);
            CQuaternion qua;
            CCylinderEntity cyl = dynamic_cast<CCylinderEntity&>(GetSpace().GetEntity(id));
            CLoopFunctions::MoveEntity(cyl.GetEmbodiedEntity(),pos,qua);

        }

    }

    previousGrab.swap(currentGrab);
    currentGrab.clear(); // clear currentgrab vector


}



/****************************************/
/*     Functions for communication      */
/****************************************/

bool CFeatureSelectionLoopFunctions::isCloseEnough(CFootBotEntity &sender, CFootBotEntity &receiver) {
    CVector2 sPos;
    CVector2 rPos;

    sPos.Set(sender.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
             sender.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());
    rPos.Set(receiver.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
             receiver.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

    float distance = sqrt(pow(rPos.GetX()-sPos.GetX(),2)+pow(rPos.GetY()-sPos.GetY(),2));

    if(distance<1 && distance != 0){
        return true;
    } else {
        if(distance==0){
           std::cout<<"myself"<<std::endl;
            return false;
        }
        std::cout<<"Ignored too far"<<std::endl;
        return false;
    }
}

void CFeatureSelectionLoopFunctions::RemoveEntity(const std::string &str_entity_id) {
    CLoopFunctions::RemoveEntity(str_entity_id);
}

int CFeatureSelectionLoopFunctions::mgenerate(int first, int last) {
    std::uniform_real_distribution<double> distribution(first, last);
    std::random_device rd;
    std::default_random_engine generator(rd());
    return distribution(generator);
}





/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CFeatureSelectionLoopFunctions, "feature_selection_loop_functions")
//
// Created by ida on 9/14/20.
//

