#include "area_cleaning_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/footbot_areacleaning/footbot_areacleaning.h>
#include <argos3/core/utility/configuration/argos_exception.h>

/****************************************/
/****************************************/

CAreaCleaningLoopFunctions::CAreaCleaningLoopFunctions() :
        m_pcFloor(NULL),
        m_pcRNG(NULL){
}

/****************************************/
/****************************************/

void CAreaCleaningLoopFunctions::Init(TConfigurationNode& t_node) {
    try {
            /* Get a pointer to the floor entity */
        m_pcFloor = &GetSpace().GetFloorEntity();
        m_pcRNG = CRandom::CreateRNG("argos");



    }
    catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
    }
}

/****************************************/
/****************************************/

void CAreaCleaningLoopFunctions::Reset() {

}

/****************************************/
/****************************************/

void CAreaCleaningLoopFunctions::Destroy() {

}

/****************************************/
/****************************************/

CColor CAreaCleaningLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) {
    if(c_position_on_plane.GetX() < -15.0f || c_position_on_plane.GetX() > 15.0f || c_position_on_plane.GetY() < -15.0f || c_position_on_plane.GetY() > 15.0f) {
        return CColor::GRAY40;      // nest area
    }
    return CColor::WHITE;           // foraging area
}

/****************************************/
/****************************************/


//
//void CAreaCleaningLoopFunctions::AddEntity(CEntity &c_entity) {
//    CSpace::TMapPerType& boxes = GetSpace().GetEntitiesByType("boxes");
//    CLEDMedium& cLEDMedium = GetSimulator().GetMedium<CLEDMedium>("leds");
//
//    for(CSpace::TMapPerType::iterator it = boxes.begin();
//        it != boxes.end();
//        ++it) {
//        CBoxEntity& cbox = *any_cast<CBoxEntity*>(it->second);
//
//        cbox.AddLED(CVector3(0.0, 0.0, 0.2), // offset
//                    CColor::BLUE);            // color
//
//        // Enable LED management for the box
//        cLEDMedium.AddEntity(cbox.GetLEDEquippedEntity().GetLED(0));
//        cbox.EnableLEDs(cLEDMedium);
//    }
//
//}
/****************************************/
/****************************************/
void CAreaCleaningLoopFunctions::PreStep() {
    CSpace::TMapPerType& m_cFootbots = GetSpace().GetEntitiesByType("foot-bot");




    for(CSpace::TMapPerType::iterator it = m_cFootbots.begin(); it != m_cFootbots.end(); ++it) {
        /* Get handle to foot-bot entity and controller */
        CFootBotEntity& sFootBot = *any_cast<CFootBotEntity *>(it->second);
        CFootBotAreaCleaning& sController = dynamic_cast<CFootBotAreaCleaning&>(sFootBot.GetControllableEntity().GetController());
        CGripperEquippedEntity m_pcGripperEquippedEntity = sFootBot.GetComponent<CGripperEquippedEntity>("gripper");

        if(m_pcGripperEquippedEntity.IsGripping()){
            //std::cout<< m_pcGripperEquippedEntity.GetGrippedEntity().GetIndex() << std::endl;
            //std::cout<< m_pcGripperEquippedEntity.GetGrippedEntity().GetParent().GetId() << std::endl;
            currentGrab.push_back(m_pcGripperEquippedEntity.GetGrippedEntity().GetParent().GetId());
        }




//        if(sController.doYouHaveAMessage()){
//            for(CSpace::TMapPerType::iterator it = m_cFootbots.begin(); it != m_cFootbots.end(); ++it) {
//                CFootBotEntity& rFootBot = *any_cast<CFootBotEntity *>(it->second);
//                CFootBotAreaCleaning& rController = dynamic_cast<CFootBotAreaCleaning&>(rFootBot.GetControllableEntity().GetController());
//                if(isCloseEnough(sFootBot,rFootBot)){
//                    std::string sMessage = sController.getMessage();
//                    rController.receivedMessage(sMessage);
//                    std::cout<< sFootBot.GetId() <<" is sending message: "<<sController.getMessage()<<" to "<<rFootBot.GetId()<<std::endl;
//
//                }
//
//            }
//            sController.messageSend();
//        }

    }

}

void CAreaCleaningLoopFunctions::PostStep() {
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
           // std::cout<<pos[0]<<" , "<<pos[1]<<" , "<<pos[2]<<std::endl;
           /* CCylinderEntity* pcCylinder = new CCylinderEntity(
                                                id,
                                                pos,
                                                CQuaternion(),
                                                true,
                                                0.1,
                                                0.2,
                                                2
            );
            if(id[0] == 'r'){
                pcCylinder ->AddLED(CVector3(0, 0, 0.2),
                                    CColor::RED);
                CLEDMedium& cLEDMedium = GetSimulator().GetMedium<CLEDMedium>("leds");
                pcCylinder->EnableLEDs(cLEDMedium);
                AddEntity(*pcCylinder);
            }
            if(id[0] == 'b'){
                pcCylinder ->AddLED(CVector3(0, 0, 0.2),
                                    CColor::BLUE);
                CLEDMedium& cLEDMedium = GetSimulator().GetMedium<CLEDMedium>("leds");
                pcCylinder->EnableLEDs(cLEDMedium);
                AddEntity(*pcCylinder);
            }
            if(id[0] == 'g'){
                pcCylinder ->AddLED(CVector3(0, 0, 0.2),
                                    CColor::GREEN);
                CLEDMedium& cLEDMedium = GetSimulator().GetMedium<CLEDMedium>("leds");
                pcCylinder->EnableLEDs(cLEDMedium);
                AddEntity(*pcCylinder);
            }*/
        }

    }

    previousGrab.swap(currentGrab);
    currentGrab.clear(); // clear currentgrab vector


}



/****************************************/
/*     Functions for communication      */
/****************************************/

bool CAreaCleaningLoopFunctions::isCloseEnough(CFootBotEntity &sender, CFootBotEntity &receiver) {
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

void CAreaCleaningLoopFunctions::RemoveEntity(const std::string &str_entity_id) {
    CLoopFunctions::RemoveEntity(str_entity_id);
}





/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CAreaCleaningLoopFunctions, "area_cleaning_loop_functions")
//
// Created by ida on 9/14/20.
//

