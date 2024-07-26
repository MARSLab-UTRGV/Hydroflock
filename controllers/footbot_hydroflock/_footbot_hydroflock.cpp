/* Include the controller definition */
#include "footbot_hydroflock.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>

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
   m_pcCamera(NULL),
   useVectorToLight(false) {}

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
   m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator          >("differential_steering");
   m_pcLight  = GetSensor  <CCI_FootBotLightSensor                    >("footbot_light");
   m_pcLEDs   = GetActuator<CCI_LEDsActuator                          >("leds");
   m_pcCamera = GetSensor  <CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
   m_pcPosition      = GetSensor    <CCI_PositioningSensor                       >("positioning");

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
      GetNodeAttribute(tSettingsNode, "target_position", m_cTargetPosition);
      GetNodeAttribute(tSettingsNode, "use_vector_to_light", useVectorToLight);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
   }

   /*
    * Other init stuff
    */
   Reset();

   m_unTicks = 0;

   log_frequency = 100;
}

/****************************************/
/****************************************/

void CFootBotHydroflock::ControlStep() {
   if (useVectorToLight) {
      SetWheelSpeedsFromVector(VectorToLight());
      // SetWheelSpeedsFromVector(VectorToLight() + FlockingVector());
   } else {
      SetWheelSpeedsFromVector(VectorToTarget());
      // SetWheelSpeedsFromVector(VectorToTarget() + FlockingVector());
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

CVector2 CFootBotHydroflock::VectorToLight() {
    /* Get light readings */
    const CCI_FootBotLightSensor::TReadings& tReadings = m_pcLight->GetReadings();
    /* Calculate a normalized vector that points to the closest light */
    CVector2 cAccum;
    for(size_t i = 0; i < tReadings.size(); ++i) {
        cAccum += CVector2(tReadings[i].Value, tReadings[i].Angle);
    }

    if (GetId() == "fb1" && m_unTicks % log_frequency == 0) {
      //   LOG << "[DEBUG] Light Readings Size: " << tReadings.size() << std::endl;
        for (size_t i = 0; i < tReadings.size(); ++i) {
            // LOG << "[DEBUG] Light Reading " << i << ": Value = " << tReadings[i].Value << ", Angle = " << tReadings[i].Angle << std::endl;
        }
      //   LOG << "[DEBUG] Accumulated Vector: " << cAccum << std::endl;
    }

    if(cAccum.Length() > 0.0f) {
        /* Make the vector long as 1/4 of the max speed */
        cAccum.Normalize();
        cAccum *= 0.25f * m_sWheelTurningParams.MaxSpeed;
        if (GetId() == "fb1" && m_unTicks % log_frequency == 0) {
            // LOG << "[DEBUG] Normalized and Scaled Vector to Light: " << cAccum << std::endl;
        }
    } else {
         if (GetId() == "fb1" && m_unTicks % log_frequency == 0) {
               // LOG << "[DEBUG] Vector to Light length is zero or negative, no normalization performed." << std::endl;
         }
    }

    if (GetId() == "fb1" && m_unTicks % log_frequency == 0) {
      //   LOG << "[DEBUG] Final Vector to Light: " << cAccum << std::endl;
    }
    return cAccum;
}


CVector2 CFootBotHydroflock::VectorToTarget() {

    // Compute the vector in the global frame
    CVector2 cVectorGlobal = m_cTargetPosition - GetCurrentPosition();

    // Get the robot's heading angle (orientation)
    CRadians cRobotOrientation = GetOrientation(); // Assume this function exists

    // Compute the cosine and sine of the orientation angle
    Real cosTheta = Cos(cRobotOrientation);
    Real sinTheta = Sin(cRobotOrientation);

    // Transform the vector to the robot's frame of reference
    Real x = cVectorGlobal.GetX() * cosTheta + cVectorGlobal.GetY() * sinTheta;
    Real y = -cVectorGlobal.GetX() * sinTheta + cVectorGlobal.GetY() * cosTheta;

    // Return the transformed vector
    return CVector2(x, y);
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

   if (GetId() == "fb1" && m_unTicks % log_frequency == 0) {
      LOG << "[DEBUG] Heading Angle: " << cHeadingAngle << std::endl;
   }

   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();

   if (GetId() == "fb1" && m_unTicks % log_frequency == 0) {
      LOG << "[DEBUG] Heading Length: " << fHeadingLength << std::endl;
   }
   
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

CVector2 CFootBotHydroflock::GetCurrentPosition(){
   const CCI_PositioningSensor::SReading& sReading = m_pcPosition->GetReading();
   return CVector2(sReading.Position.GetX(), sReading.Position.GetY());
}

CRadians CFootBotHydroflock::GetOrientation(){
   const CCI_PositioningSensor::SReading& sReading = m_pcPosition->GetReading();

   CQuaternion q = sReading.Orientation;

   // Declare CRadians for Euler angles
   CRadians cOrientation, cTemp1, cTemp2;

   // Convert quaternion to Euler angles
   q.ToEulerAngles(cOrientation, cTemp1, cTemp2);

   return cOrientation;
}

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as second argument.
 * The string is then usable in the XML configuration file to refer to this controller.
 * When ARGoS reads that string in the XML file, it knows which controller class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotHydroflock, "footbot_hydroflock_controller")