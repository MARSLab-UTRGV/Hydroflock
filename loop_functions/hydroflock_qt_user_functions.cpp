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

// void CHydroflockQTUserFunctions::AddTextOnWall(const CBoxEntity& c_entity, const std::string& text, const CVector3& position, const CColor& color){
//    glPushMatrix();

//    glTranslatef(position.GetX(), position.GetY(), position.GetZ());

//    if (c_entity.GetId() == "wall_north" || c_entity.GetId() == "wall_south") {
//       glRotatef(90.0, 1.0, 0.0, 0.0); // Rotate to align with north/south walls
//    } else if (c_entity.GetId() == "wall_east" || c_entity.GetId() == "wall_west") {
//       glRotatef(90.0, 0.0, 1.0, 0.0); // Rotate to align with east/west walls
//    }

//    glDisable(GL_LIGHTING);
//    glColor3f(color.GetRed(), color.GetGreen(), color.GetBlue());

//    glRasterPos3f(0.0f, 0.0f, 0.01f); // Slightly offset from the wall to prevent z-fighting
//    for (const char& c : text) {
//       glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, c);
//    }

//    glEnable(GL_LIGHTING);
//    glPopMatrix();

// }

// void CHydroflockQTUserFunctions::DrawOnWalls(CBoxEntity& c_entity){
//    const std::string& strId = c_entity.GetId();
//    const CVector3& position = c_entity.GetEmbodiedEntity().GetOriginAnchor().Position;
//    const CVector3& size = c_entity.GetSize();

//    if (strId == "wall_north") {
//       AddTextOnWall(c_entity, "N", position + CVector3(0, -size.GetY() / 2 - 0.01f, size.GetZ() / 2), CColor::BLACK);
//       AddTextOnWall(c_entity, "N", position + CVector3(0, size.GetY() / 2 + 0.01f, size.GetZ() / 2), CColor::BLACK);
//    } else if (strId == "wall_south") {
//       AddTextOnWall(c_entity, "S", position + CVector3(0, size.GetY() / 2 + 0.01f, size.GetZ() / 2), CColor::BLACK);
//       AddTextOnWall(c_entity, "S", position + CVector3(0, -size.GetY() / 2 - 0.01f, size.GetZ() / 2), CColor::BLACK);
//    } else if (strId == "wall_east") {
//       AddTextOnWall(c_entity, "E", position + CVector3(size.GetX() / 2 + 0.01f, 0, size.GetZ() / 2), CColor::BLACK);
//       AddTextOnWall(c_entity, "E", position + CVector3(-size.GetX() / 2 - 0.01f, 0, size.GetZ() / 2), CColor::BLACK);
//    } else if (strId == "wall_west") {
//       AddTextOnWall(c_entity, "W", position + CVector3(-size.GetX() / 2 - 0.01f, 0, size.GetZ() / 2), CColor::BLACK);
//       AddTextOnWall(c_entity, "W", position + CVector3(size.GetX() / 2 + 0.01f, 0, size.GetZ() / 2), CColor::BLACK);
//    }
// }

void CHydroflockQTUserFunctions::DrawInWorld() {
   // DrawDirections();

}


/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CHydroflockQTUserFunctions, "hydroflock_qt_user_functions")
