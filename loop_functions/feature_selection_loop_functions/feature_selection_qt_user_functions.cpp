#include "feature_selection_qt_user_functions.h"
#include <controllers/footbot_foraging/footbot_foraging.h>
#include <argos3/core/simulator/entity/controllable_entity.h>

using namespace argos;

/****************************************/
/****************************************/

CFeatureSelectionQTUserFunctions::CFeatureSelectionQTUserFunctions() {
RegisterUserFunction<CFeatureSelectionQTUserFunctions,CFootBotEntity>(&CFeatureSelectionQTUserFunctions::Draw);
}

/*******************************************************************/
/*********************use to draw the food items *******************/

void CFeatureSelectionQTUserFunctions::Draw(CFootBotEntity& c_entity) {

}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CFeatureSelectionQTUserFunctions, "feature_selection_qt_user_functions")
//
// Created by ida on 9/14/20.
//

