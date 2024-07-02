#ifndef HYDROFLOCK_QT_USER_FUNCTIONS_H
#define HYDROFLOCK_QT_USER_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#include <controllers/footbot_flocking/footbot_flocking.h>
#include <argos3/core/simulator/entity/controllable_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_camera.h>
#include <argos3/plugins/simulator/entities/box_entity.h>
#include <argos3/core/utility/math/vector3.h>
#include <GL/glut.h>

using namespace argos;

class CHydroflockQTUserFunctions : public CQTOpenGLUserFunctions {

public:

   CHydroflockQTUserFunctions();

   virtual ~CHydroflockQTUserFunctions() {}

   void DrawRobotId(CFootBotEntity& c_entity);

   void DrawDirections();

   // void DrawOnWalls(CBoxEntity& c_entity);

   void DrawInWorld();

private:

   // void AddTextOnWall(const CBoxEntity& c_entity, const std::string& text, const CVector3& position, const CColor& color);
   static bool glutInitialized;
   static void InitGLUT();
   
};

#endif
