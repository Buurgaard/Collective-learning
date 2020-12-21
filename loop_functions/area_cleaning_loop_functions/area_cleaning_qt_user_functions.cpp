#include "area_cleaning_qt_user_functions.h"
#include <controllers/footbot_foraging/footbot_foraging.h>
#include <argos3/core/simulator/entity/controllable_entity.h>

using namespace argos;

/****************************************/
/****************************************/

CAreaCleaningQTUserFunctions::CAreaCleaningQTUserFunctions() {
RegisterUserFunction<CAreaCleaningQTUserFunctions,CFootBotEntity>(&CAreaCleaningQTUserFunctions::Draw);
}

/*******************************************************************/
/*********************use to draw the food items *******************/

void CAreaCleaningQTUserFunctions::Draw(CFootBotEntity& c_entity) {

}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CAreaCleaningQTUserFunctions, "area_cleaning_qt_user_functions")
//
// Created by ida on 9/14/20.
//

