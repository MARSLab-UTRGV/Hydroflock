#ifndef HYDROFLOCK_QT_USER_FUNCTIONS_H
#define HYDROFLOCK_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

using namespace argos;

class CHydroflockQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CHydroflockQTUserFunctions();

   virtual ~CHydroflockQTUserFunctions() {}

   void Draw(CFootBotEntity& c_entity);

   void DrawInWorld();
   
};

#endif
