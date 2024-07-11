#include "hydroflock_qt_user_functions.h"

using namespace argos;

/****************************************/
/****************************************/

bool CHydroflockQTUserFunctions::glutInitialized = false;

/****************************************/
/****************************************/

CHydroflockQTUserFunctions::CHydroflockQTUserFunctions() {
   RegisterUserFunction<CHydroflockQTUserFunctions,CFootBotEntity>(&CHydroflockQTUserFunctions::DrawRobotId);
   // RegisterUserFunction<CHydroflockQTUserFunctions, CBoxEntity>(&CHydroflockQTUserFunctions::DrawOnWalls);

   if (!glutInitialized){
      InitGLUT();
      glutInitialized = true;
   }
}

void CHydroflockQTUserFunctions::InitGLUT(){
   int argc = 1;
   char *argv[1] = {(char *)"something"};
   glutInit(&argc, argv);
}

/****************************************/
/****************************************/

void CHydroflockQTUserFunctions::DrawRobotId(CFootBotEntity& c_entity) {
   /* The position of the text is expressed wrt the reference point of the footbot
    * For a foot-bot, the reference point is the center of its base.
    * See also the description in
    * $ argos3 -q foot-bot
    */
   DrawText(CVector3(0.0, 0.0, 0.3),   // position
            c_entity.GetId().c_str()); // text
}

void CHydroflockQTUserFunctions::DrawDirections(){

   // Draw text for North
   DrawText(CVector3(0.0, 5.0, 0.1), "North");
   // Draw text for South
   DrawText(CVector3(0.0, -5.0, 0.1), "South");
   // Draw text for East
   DrawText(CVector3(5.0, 0.0, 0.1), "East");
   // Draw text for West
   DrawText(CVector3(-5.0, 0.0, 0.1), "West");
}

void CHydroflockQTUserFunctions::DrawInWorld() {
   // DrawDirections();
}


/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CHydroflockQTUserFunctions, "hydroflock_qt_user_functions")
