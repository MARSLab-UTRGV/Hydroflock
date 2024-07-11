/* Include the controller definition */
#include "footbot_hydroflock.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <cmath>
#include <argos3/core/utility/math/ray2.h>
#include <sstream>

/****************************************/
/****************************************/

void CFootBotHydroflock::SWheelTurningParams::Init(TConfigurationNode& t_node) {
   try {
      TurningMechanism = NO_TURN;
      CDegrees cAngle;
      GetNodeAttribute(t_node, "hard_turn_angle_threshold", cAngle);
      HardTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "soft_turn_angle_threshold", cAngle);
      SoftTurnOnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "no_turn_angle_threshold", cAngle);
      NoTurnAngleThreshold = ToRadians(cAngle);
      GetNodeAttribute(t_node, "max_speed", MaxSpeed);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
   }
}

/****************************************/
/****************************************/

void CFootBotHydroflock::SFlockingInteractionParams::Init(TConfigurationNode& t_node) {
   try {
      GetNodeAttribute(t_node, "target_distance", TargetDistance);
      GetNodeAttribute(t_node, "gain", Gain);
      GetNodeAttribute(t_node, "exponent", Exponent);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller flocking parameters.", ex);
   }
}

/****************************************/
/****************************************/

/*
 * This function is a generalization of the Lennard-Jones potential
 */
Real CFootBotHydroflock::SFlockingInteractionParams::GeneralizedLennardJones(Real f_distance) {
   Real fNormDistExp = ::pow(TargetDistance / f_distance, Exponent);
   return -Gain / f_distance * (fNormDistExp * fNormDistExp - fNormDistExp);
}

/****************************************/
/****************************************/

CFootBotHydroflock::CFootBotHydroflock() :
   m_pcWheels(NULL),
   m_pcLight(NULL),
   m_pcLEDs(NULL),
   m_pcCamera(NULL) {}

/****************************************/
/****************************************/

void CFootBotHydroflock::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the XML tag of the
    * device whose handle we want to have. For a list of allowed values, type at the
    * command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors internally, on the basis of
    *       the lists provided the configuration file at the
    *       <controllers><footbot_diffusion><actuators> and
    *       <controllers><footbot_diffusion><sensors> sections. If you forgot to
    *       list a device in the XML and then you request it here, an error occurs.
    */
   m_pcWheels        = GetActuator  <CCI_DifferentialSteeringActuator            >("differential_steering");
   m_pcLight         = GetSensor    <CCI_FootBotLightSensor                      >("footbot_light");
   m_pcLEDs          = GetActuator  <CCI_LEDsActuator                            >("leds");
   m_pcCamera        = GetSensor    <CCI_ColoredBlobOmnidirectionalCameraSensor  >("colored_blob_omnidirectional_camera");
   m_pcPosition      = GetSensor    <CCI_PositioningSensor                       >("positioning");
   m_pcRABActuator   = GetActuator  <CCI_RangeAndBearingActuator                 >("range_and_bearing");
   m_pcRABSens       = GetSensor    <CCI_RangeAndBearingSensor                   >("range_and_bearing");

   /*
    * Parse the config file
    */
   try {
      /* Wheel turning */
      m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
      /* Flocking-related */
      m_sFlockingParams.Init(GetNode(t_node, "flocking"));
      /* Other settings */
      TConfigurationNode& tSettingsNode = GetNode(t_node, "settings");
      GetNodeAttribute(tSettingsNode, "time_step", TimeStep);
      GetNodeAttribute(tSettingsNode, "communication_test", CommunicationTest);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
   }

   /*
   * Initialize acceptable range for RAB and omni-cam correlation
   */
   m_fAcceptableRange = m_sWheelTurningParams.MaxSpeed * TimeStep;

   /*
    * Other init stuff
    */
   Reset();

   // Initialize the ticks
   m_unTicks = 0;
}

/****************************************/
/****************************************/

// modified (Ryan Luna)
void CFootBotHydroflock::ControlStep() {

   if (CommunicationTest){

      // std::stringstream ss;
      // ss << "ID: " << GetId() << ", Position: " << GetCurrentPosition() << ", Msg: Test..." << std::endl;
      // std::string msg = ss.str();

      // Message msg(GetId(), )


      // if (m_unTicks % 2 == 0) {
      //    // SendMessage(msg);
      // } else {
      //    if (GetId() == "fb1") {
      //       // OmniCameraTest();
      //       // std::vector<std::string> msgList = ReceiveMessages();

      //       for (const auto& msg : msgList) {
      //          LOG << "Received: " << msg << std::endl;
      //       }
      //    }
      // }

   }else{

      CVector2 cVectorToLight = VectorToLight();
      CVector2 cFlockingVector = FlockingVector();

      if (cVectorToLight.Length() > 0.0f){
         /* Light source is detected */
         m_cLastVectorToLight = cVectorToLight;
         SetWheelSpeedsFromVector(cVectorToLight + FlockingVector());
      } else {
         /* Light source is not detected, performm search pattern  */
         CVector2 cSearchVector = PerformSearchPattern();
         SetWheelSpeedsFromVector(cSearchVector + FlockingVector());
      }
   }

   m_unTicks++;
}

/****************************************/
/****************************************/

void CFootBotHydroflock::Reset() {
   /* Enable camera filtering */
   m_pcCamera->Enable();
   /* Set beacon color to all red to be visible for other robots */
   m_pcLEDs->SetSingleColor(12, CColor::RED);
}

/****************************************/
/****************************************/

CVector2 CFootBotHydroflock::PerformSearchPattern(){

   // Calculate the perpendicular vector to the last known vector to the light source
   CVector2 cPerpendicularVector(-m_cLastVectorToLight.GetY(), m_cLastVectorToLight.GetX());
   cPerpendicularVector.Normalize();

   // Spread out along the perpendicular vector
   static Real fSpacing = 1.0; // Define the desired spacing between robots
   size_t unRobotIndex = GetRobotIndex(); // Get unique index for this robot

   CVector2 cTargetPosition = cPerpendicularVector * unRobotIndex * fSpacing;
   cTargetPosition += GetCurrentPosition(); // Adjust target position relative to the current position

   // Calculate the heading to the target position
   CVector2 cHeading = cTargetPosition - GetCurrentPosition();
   cHeading.Normalize();
   cHeading *= 0.25f * m_sWheelTurningParams.MaxSpeed;

   return cHeading; 
}

/****************************************/
/****************************************/

CVector2 CFootBotHydroflock::VectorToLight() {
   /* Get light readings */
   const CCI_FootBotLightSensor::TReadings& tReadings = m_pcLight->GetReadings();
   /* Calculate a normalized vector that points to the closest light */
   CVector2 cAccum;
   for(size_t i = 0; i < tReadings.size(); ++i) {
      cAccum += CVector2(tReadings[i].Value, tReadings[i].Angle);
   }
   if(cAccum.Length() > 0.0f) {
      /* Make the vector long as 1/4 of the max speed */
      cAccum.Normalize();
      cAccum *= 0.25f * m_sWheelTurningParams.MaxSpeed;
   }
   return cAccum;
}

/****************************************/
/****************************************/

CVector2 CFootBotHydroflock::FlockingVector() {
   /* Get the camera readings */
   const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings = m_pcCamera->GetReadings();
   /* Go through the camera readings to calculate the flocking interaction vector */
   if(! sReadings.BlobList.empty()) {
      CVector2 cAccum;
      Real fLJ;
      size_t unBlobsSeen = 0;

      for(size_t i = 0; i < sReadings.BlobList.size(); ++i) {

         /*
          * The camera perceives the light as a yellow blob
          * The robots have their red beacon on
          * So, consider only red blobs
          * In addition: consider only the closest neighbors, to avoid
          * attraction to the farthest ones. Taking 180% of the target
          * distance is a good rule of thumb.
          */
         if(sReadings.BlobList[i]->Color == CColor::RED &&
            sReadings.BlobList[i]->Distance < m_sFlockingParams.TargetDistance * 1.80f) {
            /*
             * Take the blob distance and angle
             * With the distance, calculate the Lennard-Jones interaction force
             * Form a 2D vector with the interaction force and the angle
             * Sum such vector to the accumulator
             */
            /* Calculate LJ */
            fLJ = m_sFlockingParams.GeneralizedLennardJones(sReadings.BlobList[i]->Distance);
            /* Sum to accumulator */
            cAccum += CVector2(fLJ,
                               sReadings.BlobList[i]->Angle);
            /* Increment the blobs seen counter */
            ++unBlobsSeen;
         }
      }
      if(unBlobsSeen > 0) {
         /* Divide the accumulator by the number of blobs seen */
         cAccum /= unBlobsSeen;
         /* Clamp the length of the vector to the max speed */
         if(cAccum.Length() > m_sWheelTurningParams.MaxSpeed) {
            cAccum.Normalize();
            cAccum *= m_sWheelTurningParams.MaxSpeed;
         }
         return cAccum;
      }
      else
         return CVector2();
   }
   else {
      return CVector2();
   }
}

/****************************************/
/****************************************/

void CFootBotHydroflock::SetWheelSpeedsFromVector(const CVector2& c_heading) {
   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   /* Clamp the speed so that it's not greater than MaxSpeed */
   Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sWheelTurningParams.MaxSpeed);
   /* State transition logic */
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::HARD_TURN) {
      if(Abs(cHeadingAngle) <= m_sWheelTurningParams.SoftTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::SOFT_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) <= m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::NO_TURN;
      }
   }
   if(m_sWheelTurningParams.TurningMechanism == SWheelTurningParams::NO_TURN) {
      if(Abs(cHeadingAngle) > m_sWheelTurningParams.HardTurnOnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::HARD_TURN;
      }
      else if(Abs(cHeadingAngle) > m_sWheelTurningParams.NoTurnAngleThreshold) {
         m_sWheelTurningParams.TurningMechanism = SWheelTurningParams::SOFT_TURN;
      }
   }
   /* Wheel speeds based on current turning state */
   Real fSpeed1, fSpeed2;
   switch(m_sWheelTurningParams.TurningMechanism) {
      case SWheelTurningParams::NO_TURN: {
         /* Just go straight */
         fSpeed1 = fBaseAngularWheelSpeed;
         fSpeed2 = fBaseAngularWheelSpeed;
         break;
      }
      case SWheelTurningParams::SOFT_TURN: {
         /* Both wheels go straight, but one is faster than the other */
         Real fSpeedFactor = (m_sWheelTurningParams.HardTurnOnAngleThreshold - Abs(cHeadingAngle)) / m_sWheelTurningParams.HardTurnOnAngleThreshold;
         fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
         break;
      }
      case SWheelTurningParams::HARD_TURN: {
         /* Opposite wheel speeds */
         fSpeed1 = -m_sWheelTurningParams.MaxSpeed;
         fSpeed2 =  m_sWheelTurningParams.MaxSpeed;
         break;
      }
   }
   /* Apply the calculated speeds to the appropriate wheels */
   Real fLeftWheelSpeed, fRightWheelSpeed;
   if(cHeadingAngle > CRadians::ZERO) {
      /* Turn Left */
      fLeftWheelSpeed  = fSpeed1;
      fRightWheelSpeed = fSpeed2;
   }
   else {
      /* Turn Right */
      fLeftWheelSpeed  = fSpeed2;
      fRightWheelSpeed = fSpeed1;
   }
   /* Finally, set the wheel speeds */
   m_pcWheels->SetLinearVelocity(fLeftWheelSpeed, fRightWheelSpeed);
}

/****************************************/
/****************************************/

size_t CFootBotHydroflock::GetRobotIndex(){
   std::string strID = GetId(); // Get the robot's unique ID
   return std::stoul(strID.substr(2)); // The ID format should be in "fbX" where X is the index
}
/****************************************/
/****************************************/

CVector2 CFootBotHydroflock::GetCurrentPosition(){
   const CCI_PositioningSensor::SReading& sReading = m_pcPosition->GetReading();
   return CVector2(sReading.Position.GetX(), sReading.Position.GetY());
}


/****************************************/
/****************************************/

void CFootBotHydroflock::OmniCameraTest(){
   // Ensure m_pcPosition is not null
   if (m_pcPosition != nullptr) {
      // Get the global position and orientation of the receiving robot
      const CCI_PositioningSensor::SReading& tPositionReading = m_pcPosition->GetReading();
      CVector2 cReceiverPosition(tPositionReading.Position.GetX(), tPositionReading.Position.GetY());

      // Print the quaternion values for debugging
      LOG << "Orientation Quaternion: " << tPositionReading.Orientation << std::endl;

      // Declare CRadians for Euler angles
      CRadians cReceiverOrientation, cTemp1, cTemp2;

      // Convert quaternion to Euler angles
      tPositionReading.Orientation.ToEulerAngles(cReceiverOrientation, cTemp1, cTemp2);

      // Get the blobs detected by the camera
      const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sBlobReadings = m_pcCamera->GetReadings();
      LOG << "Blobs detected: " << sBlobReadings.BlobList.size() << std::endl;

      size_t it = 0;
      // Convert blob positions to global coordinates
      for (const auto& blob : sBlobReadings.BlobList) {
         if (blob->Color == CColor::RED) {
            CRadians adjustedBlobAngle = blob->Angle + cReceiverOrientation;
            CVector2 cBlobPosition(blob->Distance / 100 * Cos(adjustedBlobAngle), blob->Distance / 100 * Sin(adjustedBlobAngle));
            cBlobPosition += cReceiverPosition;
            LOG << "Blob " << it << " Position = " << cBlobPosition << std::endl;
         }
         it++;
      }
   } else {
      LOGERR << "Positioning sensor not initialized." << std::endl;
   }
}

/****************************************/
/****************************************/



/****************************************/
/****************************************/



/****************************************/
/****************************************/



/****************************************/
/****************************************/



/****************************************/
/****************************************/


/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as second argument.
 * The string is then usable in the XML configuration file to refer to this controller.
 * When ARGoS reads that string in the XML file, it knows which controller class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotHydroflock, "footbot_hydroflock_controller")
