#ifndef AREA_CLEANING_QT_USER_FUNCTIONS_H
#define AREA_CLEANING_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;

class CAreaCleaningQTUserFunctions : public CQTOpenGLUserFunctions {

public:

    CAreaCleaningQTUserFunctions();

    virtual ~CAreaCleaningQTUserFunctions() {}

    void Draw(CFootBotEntity& c_entity);

};

#endif
