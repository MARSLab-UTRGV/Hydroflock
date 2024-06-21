#include "hydroflock_qt_user_functions.h"
#include <controllers/footbot_flocking/footbot_flocking.h>
#include <argos3/core/simulator/entity/controllable_entity.h>

using namespace argos;

/****************************************/
/****************************************/

/****************************************/
/****************************************/

CHydroflockQTUserFunctions::CHydroflockQTUserFunctions() {
   RegisterUserFunction<CHydroflockQTUserFunctions,CFootBotEntity>(&CHydroflockQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CHydroflockQTUserFunctions::Draw(CFootBotEntity& c_entity) {
   /* The position of the text is expressed wrt the reference point of the footbot
    * For a foot-bot, the reference point is the center of its base.
    * See also the description in
    * $ argos3 -q foot-bot
    */
   DrawText(CVector3(0.0, 0.0, 0.3),   // position
            c_entity.GetId().c_str()); // text
}

void CHydroflockQTUserFunctions::DrawInWorld() {
   /* Nothing to do here */
}


/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CHydroflockQTUserFunctions, "hydroflock_qt_user_functions")
