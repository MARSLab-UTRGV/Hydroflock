/* Include the controller definition */
#include "footbot_hydroflock.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <cmath>
#include <argos3/core/utility/math/ray2.h>
#include <sstream>
#include <argos3/core/utility/math/quaternion.h>

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
      GetNodeAttribute(t_node, "GreenToRed_GainModifier", GR_GMod);
      GetNodeAttribute(t_node, "GreenToRed_TargetDistanceModifier", GR_TDMod);
      GetNodeAttribute(t_node, "RedToGreen_GainModifier", RG_GMod);
      GetNodeAttribute(t_node, "RedToGreen_TargetDistanceModifier", RG_TDMod);
      GetNodeAttribute(t_node, "RedToRed_GainModifier", RR_GMod);
      GetNodeAttribute(t_node, "RedToRed_TargetDistanceModifier", RR_TDMod);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error initializing controller flocking parameters.", ex);
   }
}

/****************************************/
/****************************************/

void CFootBotHydroflock::LogInit(){

   const std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();   // get the current time

   const std::time_t t_c = std::chrono::system_clock::to_time_t(now);   // convert the current time to a time_t object

   std::stringstream dir;  // create a string stream to hold the directory name

   dir << std::put_time(std::localtime(&t_c), "%b%d_%I-%M%p"); // log directory name format: {Month}{Day}_{Hour}-{Minute}{AM/PM}

   std::string sLogDir = m_sLogPath + dir.str();   // append the directory name to create the full path

   struct stat sb;   // struct to hold file information

   if (stat(sLogDir.c_str(), &sb) != 0){    // directory doesn't exist, create it

      if (mkdir(sLogDir.c_str(), 0777) != 0){  // create the directory, print error and terminate if it fails
         std::cerr << "Error creating directory: " << sLogDir << std::endl;
         exit(1);
      } else {
         argos::LOG << "Log Directory: " << sLogDir << std::endl;
         argos::LOG << "Log Frequency: Every " << m_unLogFrequency << " tick(s)" << std::endl;
      }

   }

   m_sLogFilePath = sLogDir + "/" + GetId() + "_adr_dev" + m_sLogFileExt;  // create the log file path
}

void CFootBotHydroflock::LogThis(const std::string& f_sMessage, const std::string& f_sFunctionName){

   m_Log.open(m_sLogFilePath, std::ios::out | std::ios::app);  // open the log file
   if (!m_Log.is_open()){  // print error and terminate if file cannot be opened
      std::cerr << "Error opening log file: " << m_sLogFilePath << std::endl;
      exit(1);
   } else {
      if (m_unTicks % m_unLogFrequency == 0 && !f_sMessage.empty()){
         std::string sHeader = "Tick: " + std::to_string(m_unTicks) + "\tIn " + f_sFunctionName + "\n";

         std::string sModString = sHeader + f_sMessage;
         size_t pos = 0;

         while ((pos = sModString.find("\n", pos)) != std::string::npos){
            sModString.insert(pos+1, "\t");
            pos += 2;
         }
         m_Log << sModString << std::endl;
         m_Log.flush();
      }
      m_Log.close();
   } 
}

/*
 * This function is a generalization of the Lennard-Jones potential
 */
Real CFootBotHydroflock::SFlockingInteractionParams::GeneralizedLennardJones( const Real& f_current_distance, const Real& f_target_distance, 
                                                                              const Real& f_gain, const Real& f_exponent) {

   // F_LJ = -G / d * ((d / D)^(2*exp) - (d / D)^exp)
   Real fNormDistExp = ::pow(f_target_distance / f_current_distance, f_exponent);
   return -f_gain / f_current_distance * (fNormDistExp * fNormDistExp - fNormDistExp);
}

/****************************************/
/****************************************/

CFootBotHydroflock::CFootBotHydroflock() :
   m_pcWheels(NULL),
   m_pcLight(NULL),
   m_pcLEDs(NULL),
   m_pcCamera(NULL),
   m_pcRABActuator(NULL),
   m_pcRABSens(NULL),
   m_pcProximity(NULL),
   m_cRab_Dsr(GetId()),
   m_cTargetPosition(CVector2()),
   m_cLastVectorToWall(CVector2()),
   m_cAlpha(20.0),
   m_bReachedTargetDistanceFromNeighbors(false),
   m_unLogFrequency(1),
   m_fWallSlope(0.0),
   m_fWallIntercept(0.0),
   m_unMaxWallPoints(100),
   m_unUpdatePrevPosFreq(10),
   m_unRegressionFreq(10){}

/****************************************/
/****************************************/

void CFootBotHydroflock::Init(TConfigurationNode& t_node) {
   /**
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
   m_pcWheels          = GetActuator  <CCI_DifferentialSteeringActuator            >("differential_steering");
   m_pcLight           = GetSensor    <CCI_FootBotLightSensor                      >("footbot_light");
   m_pcLEDs            = GetActuator  <CCI_LEDsActuator                            >("leds");
   m_pcCamera          = GetSensor    <CCI_ColoredBlobOmnidirectionalCameraSensor  >("colored_blob_omnidirectional_camera");
   m_pcPosition        = GetSensor    <CCI_PositioningSensor                       >("positioning");
   m_pcRABActuator     = GetActuator  <CCI_RangeAndBearingActuator                 >("range_and_bearing");
   m_pcRABSens         = GetSensor    <CCI_RangeAndBearingSensor                   >("range_and_bearing");
   m_pcProximity       = GetSensor    <CCI_FootBotProximitySensor                  >("footbot_proximity");
   m_pcDScanSensor     = GetSensor    <CCI_FootBotDistanceScannerSensor            >("footbot_distance_scanner");
   m_pcDScanActuator   = GetActuator  <CCI_FootBotDistanceScannerActuator          >("footbot_distance_scanner");


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
      GetNodeAttribute(tSettingsNode, "communication_test", m_bCommunicationTest);
      GetNodeAttribute(tSettingsNode, "target_position", m_cTargetPosition);
      GetNodeAttribute(tSettingsNode, "AngleThresholdInDegrees", m_cAlpha);
      GetNodeAttribute(tSettingsNode, "EnableLogging", m_bLoggingEnabled);
      GetNodeAttribute(tSettingsNode, "LogFrequency", m_unLogFrequency);
      GetNodeAttribute(tSettingsNode, "LinearRegressionFrequency", m_unRegressionFreq);
      GetNodeAttribute(tSettingsNode, "MaxWallPoints", m_unMaxWallPoints);
      GetNodeAttribute(tSettingsNode, "UpdatePreviousPositionFrequency", m_unUpdatePrevPosFreq);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("Error parsing the controller parameters.", ex);
   }

   /* Initialize acceptable range for RAB and omni-cam correlation */
   m_fAcceptableRange = m_sWheelTurningParams.MaxSpeed * TimeStep;

   /* Enable camera filtering */
   m_pcCamera->Enable();
   /* Set beacon color to all red to be visible for other robots */
   m_pcLEDs->SetSingleColor(12, CColor::GREEN);

   /* Enable distance scanner */
   m_pcDScanActuator->Enable();
   m_pcDScanActuator->SetRPM(20);   // "an OK value is 30" - ARGoS documentation in footbot_distance_scanner_actuator.h

   // Initialize the ticks
   m_unTicks = 0;

   // Initialize RAB-DSR object
   m_cRab_Dsr.Init(m_pcRABSens, m_pcRABActuator);

   // Initialize the vector of previous proximity readings
   m_vecPreviousProximityReadings = std::vector<Real>(m_pcProximity->GetReadings().size(), 0.0);

   // Initialize the state
   m_eFState = DEFAULT;

   // Initialize the log file
   LogInit();

}

/****************************************/
/****************************************/

void CFootBotHydroflock::ControlStep() {

   /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /**************** COMS TEST ****************/
   /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   if (m_bCommunicationTest){

      std::vector<CDynamicSourceRouting::Packet> incomingPackets = m_cRab_Dsr.ListenAndUpdate();

      if (GetId() == "fb1") {    // Send initial broadcast from robot 1

         bool sentRecently = false;

         std::stringstream ss;
         ss << "flood test" << std::endl;
         std::string msg = ss.str();
         CDynamicSourceRouting::Packet packet(CDynamicSourceRouting::BROADCAST, GetId(), GetCurrentPosition(), "", msg);

         if (sentRecently && m_unTicks % 10 == 0) {   // Reset the sentRecently flag ever 10 ticks
            sentRecently = false;
         }
         
         if (m_unTicks % 20 == 0 && !sentRecently) {   // Send a message every 20 ticks
            m_cRab_Dsr.AddToQueue(packet);
            sentRecently = true;
         }
      }

      for (const auto& packet: incomingPackets){
         argos::LOG << "Packet received from " << packet.GetSource() << " at " << packet.GetPosition() << " with message: " << packet.GetPayload() << std::endl;
      }
   /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /*************** END COMS TEST *************/
   /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

   }else{
      StateUpdater();
      WallPointsUsingDistanceScanner();      //! This is a test for the distance scanner, be sure to remove later.
   }
   if (m_unTicks % m_unUpdatePrevPosFreq == 0) m_cPreviousPosition = GetCurrentPosition(); // Update the previous position per the specified frequency
   m_unTicks++;
}

/****************************************/
/****************************************/

void CFootBotHydroflock::Reset() {
   /* Enable camera filtering */
   m_pcCamera->Enable();
   /* Set beacon color to all red to be visible for other robots */
   m_pcLEDs->SetSingleColor(12, CColor::GREEN);
}

/****************************************/
/****************************************/

void CFootBotHydroflock::Destroy(){

}

/****************************************/
/****************************************/

bool CFootBotHydroflock::DetectWall() {

   //TODO: This is not a reliable method differentiate between walls and robots. Need to implement a more robust method.
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   bool bWallDetected = false;
   for(size_t i = 0; i < tProxReads.size(); ++i) {
      if(tProxReads[i].Value > 0.01){
         if(!RobotInProximity(tProxReads[i].Angle)) { // Threshold value, adjust as necessary;
            bWallDetected = true;
         }
         else {
            bWallDetected = false;
            break;
         }
      }
   }
   return bWallDetected;
}

bool CFootBotHydroflock::TargetVectorUnobstructed() {

   // Get the indices of the proximity sensors relevant to the target vector
   std::vector<size_t> vRelevantSensors = GetRelevantProximitySensors(VectorToTarget());

   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();

   // Check if the relevant sensors have a non-zero reading
   for (auto i : vRelevantSensors){
      if (tProxReads[i].Value != 0 && !RobotInProximity(tProxReads[i].Angle)) return false;  // If so, and is not a robot, return false
   }

   // Else, return true
   return true;
}

bool CFootBotHydroflock::RobotInProximity(const CRadians& f_cProximityAngle){

   // Get blob list from camera
   const std::vector<CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob*>& sBlobList = m_pcCamera->GetReadings().BlobList;

   std::vector<CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob*> sFilterdBlobs;

   // Filter out blobs that are not within range of the proximity sensor
   for (const auto& blob : sBlobList){
      if (blob->Distance < 30){  // footbot diameter is 17cm and max proximity sensor range is 10cm. I added 3cm for safety...
         sFilterdBlobs.push_back(blob);
      }
   }

   // Set the threshold angle for proximity
   // TODO: This might need to be optimized??
   const CRadians cAngleThreshold = ToRadians(CDegrees(20.0f));

   for (const auto& blob : sFilterdBlobs){
      if (Abs(blob->Angle - f_cProximityAngle) <= cAngleThreshold){
         return true;
      }
   }
   return false;

}

std::vector<size_t> CFootBotHydroflock::GetRelevantProximitySensors(const CVector2& f_cTargetVector, const Real& f_cCustomThreshold){

   // Get the angle of the target vector
   CRadians cTargetAngle = f_cTargetVector.Angle();

   // List to store the indices of relevant proximity sensors
   std::vector<size_t> vRelevantSensors;

   CRadians cAngleThreshold = ToRadians(CDegrees(m_cAlpha));

   // Threshold to determine if a sensor is facing the target direction
   if (f_cCustomThreshold > -0.01f){ // if we have a positive value, this signifies that we are using a custom threshold
      cAngleThreshold = ToRadians(CDegrees(f_cCustomThreshold));
   } 

   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();

   // Iterate through the proximity sensor readings
   for (size_t i = 0; i < tProxReads.size(); ++i) {
      // Get the angle of the current proximity sensor
      CRadians cSensorAngle = tProxReads[i].Angle;

      // Check if the sensor's angle is within the threshold of the target angle
      if (Abs(cSensorAngle - cTargetAngle) <= cAngleThreshold) {
         // Add the index of the relevant sensor to the list
         vRelevantSensors.push_back(i);
      }
   }

   // Return the list of relevant sensor indices
   return vRelevantSensors;
}

CVector2 CFootBotHydroflock::CalculateWallRepulsion() {
    const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
    CVector2 cRepulsion;

    for(size_t i = 0; i < tProxReads.size(); ++i) {
        if(tProxReads[i].Value > 0.0f) {
            CRadians cAngle = tProxReads[i].Angle;
            Real fValue = tProxReads[i].Value;
            
            // Repulsion vector to maintain distance from the wall
            cRepulsion += CVector2(fValue, cAngle).Rotate(CRadians::PI);
        }
    }

    if(cRepulsion.Length() > 0.0f) {
        cRepulsion.Normalize();
        cRepulsion *= m_sWheelTurningParams.MaxSpeed; // Strong repulsive force
    }

    return cRepulsion;
}

CVector2 CFootBotHydroflock::ProjectVectorOnVector(const CVector2& f_cVectorA, const CVector2& f_cVectorB){

   /**
    * *   Proj_B(A) = [(A • B) / (B • B)] * B
    */

   // Calculate the dot product of the two vectors
   Real fDotProduct = f_cVectorA.DotProduct(f_cVectorB);

   // Calculate the magnitude of the vector B
   Real fMagnitude = f_cVectorB.Length();
   
   // Calculate the projection of vector A onto vector B
   CVector2 cProjection = f_cVectorB * (fDotProduct / (fMagnitude * fMagnitude));

   return cProjection;
}

CVector2 CFootBotHydroflock::VectorToWall(){

   std::stringstream logstream;

   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();

   AddWallPoints(tProxReads); //TODO: //!Not sure if i should do this here but it should work here for now...

   logstream << "Current Position: " << GetCurrentPosition() << std::endl;
   logstream << "Avg Wall Points: ";
   for (const auto& point : m_vWallPoints) logstream << point.GetX() << "," << point.GetY() << " ";
   logstream << std::endl;

   CVector2 cWallVector;
   bool bPrintSensor = false;
   size_t count = 0;
   m_unTicks % 20 == 0 ? bPrintSensor = true : bPrintSensor = false;

   // Calculate vector to wall
   for(size_t i = 0; i < tProxReads.size(); ++i) {

      if(tProxReads[i].Value > 0.0f) {

         CRadians cAngle = tProxReads[i].Angle;
         Real fValue = tProxReads[i].Value;

         logstream   << "Sensor angle: " << ToDegrees(cAngle) << std::endl
                     << "Sensor value: " << fValue << std::endl;

         cWallVector+= CVector2(fValue, cAngle);

         count++;
      }
   }

   if (m_bLoggingEnabled) HFLOG(logstream.str());
   return cWallVector/count;
}

CVector2 CFootBotHydroflock::ReorientTowardsWall(){

   std::stringstream logstream;

   // Get current pose
   CVector2 cCurrentPosition = GetCurrentPosition();
   CRadians cCurrentHeading = GetOrientation();

   logstream   << "Current position: " << cCurrentPosition           << std::endl
               << "Current heading: "  << ToDegrees(cCurrentHeading) << std::endl;

   // Translate the last vector to wall to the global frame
   CVector2 cNormal_Global = m_cLastVectorToWall;
   cNormal_Global.Rotate(m_cLastWallContactHeading);
   cNormal_Global.Normalize();
   cNormal_Global *= 0.25f * m_sWheelTurningParams.MaxSpeed;

   logstream   << "Last vector to wall (local): "  << ToDegrees(m_cLastVectorToWall.Angle()) << std::endl
               << "Last wall contact heading: "    << ToDegrees(m_cLastWallContactHeading)   << std::endl
               << "cNormal_Global: "               << ToDegrees(cNormal_Global.Angle())      << std::endl;

   // Calculate the vector to the last wall contact position
   CVector2 cVectorToLastPosition = m_cLastWallContactPosition - cCurrentPosition;
   cVectorToLastPosition.Normalize();
   cVectorToLastPosition *= 0.25f * m_sWheelTurningParams.MaxSpeed;

   logstream   << "Last wall contact position: "         << m_cLastWallContactPosition       << std::endl
               << "Vector to last position (angle): "    << cVectorToLastPosition.Angle()    << std::endl
               << "Vector to last position (length): "   << cVectorToLastPosition.Length()   << std::endl;

   // Project the vector to the last wall contact position onto the global normal vector
   CVector2 cNormalDisplacement_Global = ProjectVectorOnVector(cVectorToLastPosition, cNormal_Global);

   // Ensure the normal displacement vector points towards the wall
   if (cNormalDisplacement_Global.DotProduct(cNormal_Global) < 0.0f) cNormalDisplacement_Global = -cNormalDisplacement_Global;

   logstream   << "Normal displacement global (angle): "  << cNormalDisplacement_Global.Angle()   << std::endl
               << "Normal displacement global (length): " << cNormalDisplacement_Global.Length()  << std::endl;

   // Translate back into robot frame
   CVector2 cNormalDisplacement = cNormalDisplacement_Global;
   cNormalDisplacement.Rotate(-cCurrentHeading);

   logstream   << "Normal displacement local (angle): "  << cNormalDisplacement.Angle()   << std::endl
               << "Normal displacement local (length): " << cNormalDisplacement.Length()  << std::endl;

   if (m_bLoggingEnabled) HFLOG(logstream.str());
   // if (GetId() == "fb4") argos::LOG << logstream.str() << std::endl;
   return cNormalDisplacement;
}

void CFootBotHydroflock::AddWallPoints(const CCI_FootBotProximitySensor::TReadings& f_cProximityReadings){

   CVector2 cWallVector;
   CRadians cSumOfAngles;
   Real fMaxSensorValue = 0;
   Real fSumOfValues = 0;
   size_t count = 0;

   /**
    * Loop through proximity readings and only get the active ones
    */
   for (size_t i = 0; i < f_cProximityReadings.size(); ++i){
      if (f_cProximityReadings[i].Value > 0.0f){

         CRadians cAngle = f_cProximityReadings[i].Angle;   // Get the angle of the sensor
         Real fValue = f_cProximityReadings[i].Value;       // Get the value of the sensor
         cSumOfAngles += cAngle;                            // Add the angle to the sum of angles
         // fSumOfValues += fValue;                            // Add the value to the sum of values  //! Trying average instead of max...maybe the average will be more stable
         count++;                                          // Increment the count of active sensors (used to get average angle)

         
         if (fValue > fMaxSensorValue) {
            fMaxSensorValue = fValue;                     // Only use the maximum sensor value
         }
      }
   }
   
   if (count > 0){

      CRadians cAngleAverage = cSumOfAngles / count;
      
      /**
       * Convert the sensor value to a distance value (max distance is 10cm) in cm. 
       * Sensor values range from 0 to 1 so we multiply by 10 to scale this to 0cm to 10cm.
       * The sensor value is inversely proportional to the distance so we subtract the value from 10 to get the distance. (e.g. 1 is touching and 0 is 10cm away, < 0 is no detection)
       * 
       * We add 8.5 to the distance value to get the distance from the center of the robot to the wall.
       * 
       * We divide the distance by 100 to convert it to meters.
      */
      Real fDistance = 10 - (fMaxSensorValue * 10); // scale the sensor value to 0-10cm //! ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      // Real fDistance = 10 - (fSumOfValues * 10); // scale the sensor value to 0-10cm
      fDistance += 8.5; // add the distance from the center of the robot to the sensor
      fDistance /= 100.0f; // convert to meters

      CVector2 cCurrentPosition = GetCurrentPosition();
      CRadians cCurrentHeading = GetOrientation();

      /**
       * Get the point on the wall in the robot's frame of reference
       */
      Real cPointLocalX = fDistance * Cos(cAngleAverage);
      Real cPointLocalY = fDistance * Sin(cAngleAverage);

      /**
       * Translate into the global frame of reference
       * 
       * Rotate the point to the global frame of reference using a rotation matrix     [   cos(theta) -sin(theta)  ] 
       *                                                                               [   sin(theta)  cos(theta)  ]
       * The point is rotated by the current heading of the robot.
       */
      Real cXRotated = cPointLocalX * Cos(cCurrentHeading) - cPointLocalY * Sin(cCurrentHeading); // x' = xcos(theta) - ysin(theta)
      Real cYRotated = cPointLocalX * Sin(cCurrentHeading) + cPointLocalY * Cos(cCurrentHeading); // y' = xsin(theta) + ycos(theta)

      /**
       * Translate the point to the global frame of reference by adding the current position of the robot.
       */
      Real cPointGobalX = cXRotated + cCurrentPosition.GetX(); // x'' = x' + x0
      Real cPointGobalY = cYRotated + cCurrentPosition.GetY(); // y'' = y' + y0

      CVector2 cPoint(cPointGobalX, cPointGobalY);

      m_vWallPoints.push_back(cPoint); // Add the point to the list (queue) of wall points

      if (m_vWallPoints.size() > m_unMaxWallPoints) m_vWallPoints.erase(m_vWallPoints.begin()); // Remove the oldest point if the queue exceeds the maximum size
   } 
}

void CFootBotHydroflock::WallPointsUsingDistanceScanner(){

   // Get blob list from camera
   const std::vector<CCI_ColoredBlobOmnidirectionalCameraSensor::SBlob*>& sBlobList = m_pcCamera->GetReadings().BlobList;

   /** 
    * A map of the readings of the ds short sensor readings in the following format:
    * angle - value
    *
    * The value is the distance wrt to a perceived obstacle.
    * If the value is -1 it means that the sensor is saturated (the obstacle is too close, i.e. < 4cm from the robot border)
    * If the value is -2 it means that the sensor is empy (obstacle too far / no obstacle, i.e. > 30cm from the robot border)
    */
   const CCI_FootBotDistanceScannerSensor::TReadingsMap& tDScanShortReadings = m_pcDScanSensor->GetShortReadingsMap();
   /**
    * A map of the readings of the ds long sensor readings in the following format:
    * angle - value
    *
    * The value is the distance wrt to a perceived obstacle.
    * If the value is -1 it means that the sensor is saturated (the obstacle is too close, i.e. < 20cm from the robot border)
    * If the value is -2 it means that the sensor is empy (obstacle too far / no obstacle, i.e. > 150cm from the robot border)
    */
   const CCI_FootBotDistanceScannerSensor::TReadingsMap& tDScanLongReadings = m_pcDScanSensor->GetLongReadingsMap();

   const CRadians temporaryAngleThreshold = ToRadians(CDegrees(20.0f)); // temporary threshold for filtering out the same angle //! this threshold is arbitrary, might need to be adjusted

   bool skip = false;

   for (auto& shortReading : tDScanShortReadings){
      CRadians cAngle = shortReading.first;
      Real fValue = shortReading.second;

      for (const auto& blob : sBlobList){
         if (Abs(blob->Angle - cAngle) <= temporaryAngleThreshold){ 
            skip = true;
            break;
         }
      }

      if (skip){
         skip = false;
         continue;
      }

               //? Just want to note that I'm not sure what happens if we get the same angle more than once. Judging by the TReadingsMap type, it should
               //? be overwritten so we probably won't have to worry about checking for that.

      if (fValue > 0.0f){
         Real fDistance = fValue;
         fDistance /= 100.0f; // convert to meters

         CVector2 cCurrentPosition = GetCurrentPosition();
         CRadians cCurrentHeading = GetOrientation();

         /**
          * Get the point on the wall in the robot's frame of reference
          */
         Real cPointLocalX = fDistance * Cos(cAngle);
         Real cPointLocalY = fDistance * Sin(cAngle);

         /**
          * Translate into the global frame of reference
          * 
          * Rotate the point to the global frame of reference using a rotation matrix     [   cos(theta) -sin(theta)  ] 
          *                                                                               [   sin(theta)  cos(theta)  ]
          * The point is rotated by the current heading of the robot.
          */
         Real cXRotated = cPointLocalX * Cos(cCurrentHeading) - cPointLocalY * Sin(cCurrentHeading); // x' = xcos(theta) - ysin(theta)
         Real cYRotated = cPointLocalX * Sin(cCurrentHeading) + cPointLocalY * Cos(cCurrentHeading); // y' = xsin(theta) + ycos(theta)

         /**
          * Translate the point to the global frame of reference by adding the current position of the robot.
          */
         Real cPointGobalX = cXRotated + cCurrentPosition.GetX(); // x'' = x' + x0
         Real cPointGobalY = cYRotated + cCurrentPosition.GetY(); // y'' = y' + y0

         CVector2 cPoint(cPointGobalX, cPointGobalY);

         m_vWallPointsDS.push_back(cPoint); // Add the point to the list (queue) of wall points

         if (m_vWallPointsDS.size() > m_unMaxWallPoints) m_vWallPointsDS.erase(m_vWallPointsDS.begin()); // Remove the oldest point if the queue exceeds the maximum size
      }
   }

   for (auto& longReading : tDScanLongReadings){
      CRadians cAngle = longReading.first;
      Real fValue = longReading.second;

      for (const auto& blob : sBlobList){
         if (Abs(blob->Angle - cAngle) <= temporaryAngleThreshold){
            fValue = 0.0f;
            skip = true;
            break;
         }
      }

      if (skip){
         skip = false;
         continue;
      }

      if (fValue > 0.0f){
         Real fDistance = fValue;
         fDistance /= 100.0f; // convert to meters

         CVector2 cCurrentPosition = GetCurrentPosition();
         CRadians cCurrentHeading = GetOrientation();

         /**
          * Get the point on the wall in the robot's frame of reference
          */
         Real cPointLocalX = fDistance * Cos(cAngle);
         Real cPointLocalY = fDistance * Sin(cAngle);

         /**
          * Translate into the global frame of reference
          * 
          * Rotate the point to the global frame of reference using a rotation matrix     [   cos(theta) -sin(theta)  ] 
          *                                                                               [   sin(theta)  cos(theta)  ]
          * The point is rotated by the current heading of the robot.
          */
         Real cXRotated = cPointLocalX * Cos(cCurrentHeading) - cPointLocalY * Sin(cCurrentHeading); // x' = xcos(theta) - ysin(theta)
         Real cYRotated = cPointLocalX * Sin(cCurrentHeading) + cPointLocalY * Cos(cCurrentHeading); // y' = xsin(theta) + ycos(theta)

         /**
          * Translate the point to the global frame of reference by adding the current position of the robot.
          */
         Real cPointGobalX = cXRotated + cCurrentPosition.GetX(); // x'' = x' + x0
         Real cPointGobalY = cYRotated + cCurrentPosition.GetY(); // y'' = y' + y0

         CVector2 cPoint(cPointGobalX, cPointGobalY);

         m_vWallPointsDS.push_back(cPoint); // Add the point to the list (queue) of wall points

         if (m_vWallPointsDS.size() > m_unMaxWallPoints) m_vWallPointsDS.erase(m_vWallPointsDS.begin()); // Remove the oldest point if the queue exceeds the maximum size
      }
   }

   std::stringstream logstream;
   logstream << "Current Position: " << GetCurrentPosition() << std::endl;
   logstream << "DS Wall Points: ";
   for (const auto& point : m_vWallPointsDS) logstream << point.GetX() << "," << point.GetY() << " ";
   logstream << std::endl;

   if (m_bLoggingEnabled) HFLOG(logstream.str());
}

CVector2 CFootBotHydroflock::CalculateTangentialMovement(const CVector2& f_cFlockingVector) {

   std::stringstream logstream;

   /**
    * We want to calculate the tangential movement along the wall during wall dispersion.
    * 
    * Projecting the flocking vector onto the tangential direction will prevent the flocking vector
    * from pushing the robot into or away from the wall. It will only push in the direction tangent to the wall.
    */

   CVector2 cTangential;
   CVector2 cNormal = VectorToWall().Rotate(CRadians::PI);
   CVector2 cCombinedVector;
   bool bOffWall = !DetectWall();

   logstream << "Off wall: " << bOffWall << std::endl;

   if (bOffWall){ // if the robot is no longer touching the wall
      cNormal = ReorientTowardsWall();

   } else {
      m_cLastVectorToWall = cNormal;
      m_cLastVectorToWall.Rotate(CRadians::PI); // Rotate the vector to point back to the wall for term consistency vector "TO" wall.
      m_cLastWallContactPosition = GetCurrentPosition();
      m_cLastWallContactHeading = GetOrientation();
   }

   cTangential = cNormal;
   cTangential.Rotate(CRadians::PI_OVER_TWO);
   cTangential.Normalize();
   cTangential *= 0.25f * m_sWheelTurningParams.MaxSpeed;

   // Enforce the distance constraint if the robot is touching the wall
   if (!bOffWall){

      Real fDesiredSensorValue = 0.25;  // Desired sensor value for 7.5 cm away from the wall
      Real fCurrentSensorValue = cNormal.Length();
      CRadians fCurrentAngle = cNormal.Angle();

      logstream   << "Initial cNormal (angle): "   << ToDegrees(cNormal.Angle()) << std::endl
                  << "Initial cNormal (length): "  << cNormal.Length()           << std::endl;

      Real fDistanceError = fCurrentSensorValue - fDesiredSensorValue;
      cNormal = CVector2(fDistanceError, fCurrentAngle);

      if (fDistanceError < 0.0f) cNormal.Rotate(CRadians::PI);
   }

   // // Normalize the normal vector //? idk if i should be doing this but i'm testing it...
   // cNormal.Normalize();
   // Real fScalingFactor = 0.25f * cTangential.Length();   // scale the normal vector to 25% of the tangential vector
   // cNormal = CVector2(cNormal.Length()*fScalingFactor, cNormal.Angle());   // we do it like this to make sure we keep the same angle
   //                                                                         //? sometimes the angle changes when multiplying the vector by a scalar

   // Project the flocking vector onto the tangential direction
   CVector2 cProjectedFlockingVector = ProjectVectorOnVector(FlockingVector(), cTangential);

   // Let the tangent vector direction be the direction of the projected flocking vector //? Trying this out...
   cTangential = CVector2(cTangential.Length(), cProjectedFlockingVector.Angle());

   // Combine the tangential vector, the projected flocking vector, and the normal vector
   cCombinedVector = cTangential + cProjectedFlockingVector + cNormal;

   // Ensure the combined vector does not exceed the maximum speed
   if(cCombinedVector.Length() > m_sWheelTurningParams.MaxSpeed) {
      cCombinedVector.Normalize();
      cCombinedVector *= m_sWheelTurningParams.MaxSpeed;
   }

   logstream   << "Final cNormal (angle): " << ToDegrees(cNormal.Angle()) << std::endl
               << "Final cNormal (length): " << cNormal.Length() << std::endl
               << "cTangential (angle): " << ToDegrees(cTangential.Angle()) << std::endl
               << "cTangential (length): " << cTangential.Length() << std::endl
               << "cProjectedFlockingVector (angle): " << ToDegrees(cProjectedFlockingVector.Angle()) << std::endl
               << "cProjectedFlockingVector (length): " << cProjectedFlockingVector.Length() << std::endl
               << "cCombinedVector (angle): " << ToDegrees(cCombinedVector.Angle()) << std::endl
               << "cCombinedVector (length): " << cCombinedVector.Length() << std::endl;


   if(m_bLoggingEnabled) HFLOG(logstream.str());
   // if(GetId()=="fb4") argos::LOG << logstream.str() << std::endl;
   return cCombinedVector;
}

bool CFootBotHydroflock::CornerDetected(){
   
   if (OuterCornerDetected() || InnerCornerDetected()){
      return true;
   }

   return false;
}

void CFootBotHydroflock::DoLinearRegression(){

   std::stringstream logstream;

   if (!m_bInitWallRegression) m_bInitWallRegression = true;

   /**
    * Separate the wall points into separate lists for x and y coordinates
    * 
    * This is needed to use the linear regression function in the GSL library
    * 
    * ? Currently using the moving average of the wall points
    */
   std::vector<Real> vXList, vYList;
   // std::vector<CVector2> m_vWallPointsMovingAverage = GetWallPointsMovingAverage(70); //TODO: Implement dynamic window size
   // logstream << "Wall Points Moving Average: ";
   // for (const auto& point : m_vWallPointsMovingAverage){
   for (const auto& point : m_vWallPointsDS){
      // logstream << point.GetX() << "," << point.GetY() << " ";
      vXList.push_back(point.GetX());
      vYList.push_back(point.GetY());
   }
   logstream << std::endl;

   /**
    * Intercept (c0): The y-intercept of the regression line.
    * 
    * Slope (c1): The slope of the regression line.
    * 
    * Variance and Covariance (cov00, cov01, cov11): These give information about the uncertainty in the slope and intercept.
    * 
    * Sum of Squares (sumsq): Indicates the overall error in the fit.
    */
   double m, b, cov00, cov01, cov11, sumsq;

   // Calculate the linear regression of the wall points using the GSL library
   gsl_fit_linear(vXList.data(), 1, vYList.data(), 1, m_vWallPointsDS.size(), &m, &b, &cov00, &cov01, &cov11, &sumsq);

   m_fWallSlope = m;        // Update slope of the regression line
   m_fWallIntercept = b;    // Update intercept of the regression line

   logstream << "Slope: " << m_fWallSlope << std::endl;
   logstream << "Intercept: " << m_fWallIntercept << std::endl;

   if (m_bLoggingEnabled) HFLOG(logstream.str());
}

std::vector<CVector2> CFootBotHydroflock::GetWallPointsMovingAverage(int window_size){

   /**
    * Get the moving average of the wall points and return the smoothed points
    */

   std::vector<CVector2> smoothed_points;
   int half_window = window_size / 2;

   for(size_t i = 0; i < m_vWallPoints.size(); ++i){
      CVector2 sum(0,0);
      int count = 0;

      // Calculate the averagte of the points within the window
      for (int j = -half_window; j <= half_window; ++j){
         int index = i + j;
         if (index >= 0 && index < m_vWallPoints.size()){
            sum += m_vWallPoints[index];
            count++;
         }
      }

      smoothed_points.push_back(sum / count);
   }

   return smoothed_points;

}

bool CFootBotHydroflock::OuterCornerDetected(){

   if (!m_bInitWallRegression && m_vWallPoints.size() > m_unMinWallPoints){
      DoLinearRegression();
   } else if (m_unTicks % m_unRegressionFreq == 0 && m_vWallPoints.size() > m_unMinWallPoints){
      DoLinearRegression();
   }

   /**
    * Get the current and previous position of the robot
    * 
    * x1, y1: Previous position
    * x2, y2: Current position
    */
   Real x1 = m_cPreviousPosition.GetX();
   Real y1 = m_cPreviousPosition.GetY();
   Real x2 = GetCurrentPosition().GetX();
   Real y2 = GetCurrentPosition().GetY();

   // Calculate the y values of the previous and current positions on the regression line
   Real y1_on_line = m_fWallSlope * x1 + m_fWallIntercept;
   Real y2_on_line = m_fWallSlope * x2 + m_fWallIntercept;

   // Check for intersection
   if ((y1 > y1_on_line && y2 < y2_on_line) || (y1 < y1_on_line && y2 > y2_on_line)) {
        return true;
   }

   return false;

}

bool CFootBotHydroflock::InnerCornerDetected(){

   //TODO: Transition to gradient-based approach for inner corner detection
   /**
    * * Empirical testing through plotting collected data on the gradients of the proximity sensor readings
    */

   /**
    * Current implementation is a straightforward approach checking if not facing the wall (previously zero)
    * become non-zero while other relevant sensors (not facing the wall) are still zero.
    * 
    * 
    *                            _______________________________   
    *                Wall 1 --> |_______________________________| 
    *                                              \   /      | |
    *   Sensors facing Wall 1 (value: non-zero)-->  \ /       | |
    *                                      Robot--> (0)__ __ <|-|---- Sensor facing away from Wall 1  (value: zero)
    *                                                 \       | |                                     (facing Wall 2)
    *    Sensor facing away from Wall 1 (non-zero)-->  \      |_| <-- Wall 2                          (previously non-zero)
    */
   
   const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
   std::vector<size_t> vIrrelevantSensors = GetRelevantProximitySensors(VectorToWall());  // The irrelevant sensor are those facing the wall
   bool bCornerDetected = false;

   // Get relevant sensors
   std::vector<size_t> vRelevantSensors;
   for (size_t i = 0; i < tProxReads.size(); ++i){
      if (std::find(vIrrelevantSensors.begin(), vIrrelevantSensors.end(), i) == vIrrelevantSensors.end()){
         vRelevantSensors.push_back(i);
      }
   }

   // loop through the relevant sensors
   for (size_t i : vRelevantSensors) {
      // Check if the current sensor reading is non-zero and the previous was zero
      if (m_vecPreviousProximityReadings[i] == 0.0f && tProxReads[i].Value > 0.0f) {

            /**
             *!   I don't think I need to check adjacent sensors for inner corner detection
             *!   I will leave this here for now and remove it later if it is not needed.
             *
             **   I am essentially just checking if any other sensors detect a wall. 😄
             *
             *?   I might need to specifically check if sensors facing DIRECTLY away from the 
             *?   wall are non-zero. (Hallway scenario)
             */

            // size_t nextIndex = (i + 1) % tProxReads.size();    // Get the index of the next sensor (goes back to 0 if at the end)
            // size_t prevIndex = (i == 0) ? tProxReads.size() - 1 : i - 1;   // Get the index of the previous sensor (goes to the last index if at 0)

            // // Check if adjacent sensors still have non-zero readings
            // if (tProxReads[nextIndex].Value > 0.0f || tProxReads[prevIndex].Value > 0.0f) {
            //    bCornerDetected = true;
            //    break;
            // }

         bCornerDetected = true;
         break;
      }
   }

   // Update previous readings
   for (size_t i : vRelevantSensors) {
      m_vecPreviousProximityReadings[i] = tProxReads[i].Value;
   }

   return bCornerDetected;

}

/****************************************/
/****************************************/

void CFootBotHydroflock::SetFlockingState(const FlockingState& f_state){

   std::stringstream logstream;

   switch(f_state){

      case DEFAULT:
         if (!m_bPrintState){
            logstream << "Setting state to DEFAULT" << std::endl;
            m_bPrintState = true;
         } 
         m_pcLEDs->SetSingleColor(12, CColor::GREEN);
         NormalFlocking();
         m_eFState = DEFAULT;
         break;

      case WALL_DISPERSION:
         if (!m_bPrintState){
            logstream << ": Setting state to WALL_DISPERSION" << std::endl;
            m_bPrintState = true;
         }
         m_pcLEDs->SetSingleColor(12, CColor::RED);
         WallDispersion();
         m_eFState = WALL_DISPERSION;
         break;

      case WALL_FOLLOWING:
         if (!m_bPrintState){
            logstream << ": Setting state to WALL_FOLLOWING" << std::endl;
            m_bPrintState = true;
         }
         m_pcLEDs->SetSingleColor(12, CColor::BLUE);
         WallFollowing();
         m_eFState = WALL_FOLLOWING;
         break;

      case AGGREGATOR:
         if (!m_bPrintState){
            logstream << ": Setting state to AGGREGATOR" << std::endl;
            m_bPrintState = true;
         }
         m_pcLEDs->SetSingleColor(12, CColor::MAGENTA);
         Aggregator();
         m_eFState = AGGREGATOR;
         break;

      case AGGREGATEE:
         if (!m_bPrintState){
            logstream << ": Setting state to AGGREGATEE" << std::endl;
            m_bPrintState = true;
         }
         m_pcLEDs->SetSingleColor(12, CColor::PURPLE);
         Aggregatee();
         m_eFState = AGGREGATEE;
         break;
   }

   if(m_bLoggingEnabled) HFLOG(logstream.str());
}

void CFootBotHydroflock::StateUpdater(){

   std::stringstream logstream;

   GetNeighborStates();
   
   switch (m_eFState){

      case DEFAULT:  // Green

         if (DetectWall()){

            m_bPrintState = false;
            logstream << "Initial vector to wall: " << ToDegrees(VectorToWall().Angle()) << std::endl;
            SetFlockingState(WALL_DISPERSION);
         } else {
            SetFlockingState(DEFAULT);
         }
         break;

      case WALL_DISPERSION: // Red

         // if(m_bHasMagentaNeighbor || m_bHasPurpleNeighbor){

         //    m_bPrintState = false;
         //    SetFlockingState(AGGREGATEE);
         
         // } else 
         if (OuterCornerDetected()){

            // if (TargetVectorUnobstructed()){

            //    m_bPrintState = false;
            //    SetFlockingState(AGGREGATOR);

            // } else {
            //    //TODO: Need to navigate flock around corner
            // }

            logstream << "Detected outer corner: " << std::endl;
            argos::LOG << GetId() << ": Detected outer corner" << std::endl;
         
         } 
         // else if (InnerCornerDetected()){

         //    logstream << "Detected inner corner: " << std::endl;

         
         // } 
         // else if (m_bReachedTargetDistanceFromNeighbors){

         //    m_bPrintState = false;
         //    SetFlockingState(WALL_FOLLOWING);
         
         // } 
         else {
            SetFlockingState(WALL_DISPERSION);
         }
         
         break;

      default:
         break;
   }

   if(m_bLoggingEnabled) HFLOG(logstream.str());
   // argos::LOG << GetId() << ": " << logstream.str() << std::endl;
}

void CFootBotHydroflock::GetNeighborStates(){

   const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sReadings = m_pcCamera->GetReadings();

   size_t iRedCount=0, iGreenCount=0, iPurpleCount=0, iBlueCount=0, iMagentaCount=0;

   for(int i = 0; i < sReadings.BlobList.size(); i++){
      // We only consider neighbors within 180% of the target distance like the FlockingVector function.
      if (sReadings.BlobList[i]->Distance < m_sFlockingParams.TargetDistance * 1.80f){
         if(sReadings.BlobList[i]->Color == CColor::RED){
            iRedCount++;
         } else if(sReadings.BlobList[i]->Color == CColor::GREEN){
            iGreenCount++;
         } else if(sReadings.BlobList[i]->Color == CColor::PURPLE){
            iPurpleCount++;
         } else if(sReadings.BlobList[i]->Color == CColor::BLUE){
            iBlueCount++;
         } else if(sReadings.BlobList[i]->Color == CColor::MAGENTA){
            iMagentaCount++;
         }
      }
   }

   m_bHasRedNeighbor = iRedCount > 0;
   m_bHasGreenNeighbor = iGreenCount > 0;
   m_bHasPurpleNeighbor = iPurpleCount > 0;
   m_bHasBlueNeighbor = iBlueCount > 0;
   m_bHasMagentaNeighbor = iMagentaCount > 0;
}

void CFootBotHydroflock::NormalFlocking(){
   
      // Get the vector to the target
      CVector2 cVectorToTarget = VectorToTarget();
   
      // Get the flocking vector
      CVector2 cFlockingVector = FlockingVector();
   
      // Combine the two vectors
      CVector2 cCombinedVector = cVectorToTarget + cFlockingVector;
   
      // Set the wheel speeds based on the combined vector
      SetWheelSpeedsFromVector(cCombinedVector);
}

void CFootBotHydroflock::WallDispersion(){

   // Move along the wall and spread out using Lennard-Jones potential
   CVector2 cTangentialMovementVector = CalculateTangentialMovement(FlockingVector());

   if (cTangentialMovementVector.Length() <= 0.0f){
      LOGERR << GetId() << " Error: Tangential movement vector is zero" << std::endl;
   }

   SetWheelSpeedsFromVector(cTangentialMovementVector);
}

void CFootBotHydroflock::WallFollowing(){

}

void CFootBotHydroflock::Aggregator(){

}

void CFootBotHydroflock::Aggregatee(){

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
          * Consider only the closest neighbors, to avoid
          * attraction to the farthest ones. Taking 180% of the target
          * distance is a good rule of thumb.
          */
         Real fcutOffDistance = m_sFlockingParams.TargetDistance * 1.80f;
         Real fblobDistance = sReadings.BlobList[i]->Distance;

         CColor ledColor = sReadings.BlobList[i]->Color;

         if (fblobDistance < fcutOffDistance){

            /** 
             * * Take the blob distance and angle
             * * With the distance, calculate the Lennard-Jones interaction force
             * * Form a 2D vector with the interaction force and the angle
             * * Sum such vector to the accumulator
             * 
             * The flocking behavior is divided into states
             * Case dependent on the LED color of neighbors
             */
            switch (m_eFState){

               case DEFAULT:

                  if (ledColor == CColor::GREEN){  // Neighbor is in DEFAULT state

                     /**
                      * * Normal Flocking Behavior
                      */

                     /* Calculate LJ with modified Gain and TargetDistance*/
                     Real fTargetDistance = m_sFlockingParams.TargetDistance;
                     Real fGain = m_sFlockingParams.Gain;
                     Real fExponent = m_sFlockingParams.Exponent;

                     fLJ = m_sFlockingParams.GeneralizedLennardJones(   fblobDistance,    fTargetDistance, 
                                                                                          fGain,
                                                                                          fExponent);

                     // Check if the robot is within +- 20% of the target distance from neighbors
                     if (  fblobDistance < fTargetDistance * 1.20f && 
                           fblobDistance > fTargetDistance * 0.80f){
                        m_bReachedTargetDistanceFromNeighbors = true;
                     } else {
                        m_bReachedTargetDistanceFromNeighbors = false;
                     }

                     /* Sum to accumulator */
                     cAccum += CVector2(fLJ,sReadings.BlobList[i]->Angle);
                     /* Increment the blobs seen counter */
                     ++unBlobsSeen;

                  } else if (ledColor == CColor::RED){   // Neighbor is in WALL_DISPERSION state

                     /**
                      * * GREEN -> RED Flocking Behavior
                      * 
                      * Robots in DEFAULT flocking state (GREEN) should have weak or low repulsion from 
                      * robots in Wall Dispersion state (RED). This should allow the DEFAULT state robots
                      * to reach the wall without being pushed away too strongly by robots in WALL_DISPERSION state.
                      */
                     
                     /* Calculate LJ with modified Gain and TargetDistance*/
                     Real fTargetDistance = m_sFlockingParams.TargetDistance * m_sFlockingParams.GR_TDMod;
                     Real fGain = m_sFlockingParams.Gain * m_sFlockingParams.GR_GMod;
                     Real fExponent = m_sFlockingParams.Exponent;

                     fLJ = m_sFlockingParams.GeneralizedLennardJones(   fblobDistance,    fTargetDistance, 
                                                                                          fGain,
                                                                                          fExponent);

                     // Check if the robot is within +- 20% of the target distance from neighbors
                     if (  fblobDistance < fTargetDistance * 1.20f && 
                           fblobDistance > fTargetDistance * 0.80f){
                        m_bReachedTargetDistanceFromNeighbors = true;
                     } else {
                        m_bReachedTargetDistanceFromNeighbors = false;
                     }

                     /* Sum to accumulator */
                     cAccum += CVector2(fLJ,sReadings.BlobList[i]->Angle);
                     /* Increment the blobs seen counter */
                     ++unBlobsSeen;
                  }

                  break;

               case WALL_DISPERSION:

                  if (ledColor == CColor::GREEN){

                     /**
                      * * RED -> GREEN Flocking Behavior
                      * 
                      * Robots in WALL_DISPERSION flocking state (RED) should have weak or low repulsion from 
                      * robots in DEFAULT state (GREEN). This should preven the WALL_DISPERSION state robots
                      * from being pushed into the wall by robots in DEFAULT state.
                      */
                     
                     /* Calculate LJ with modified Gain and TargetDistance*/
                     Real fTargetDistance = m_sFlockingParams.TargetDistance * m_sFlockingParams.RG_TDMod;
                     Real fGain = m_sFlockingParams.Gain * m_sFlockingParams.RG_GMod;
                     Real fExponent = m_sFlockingParams.Exponent;

                     fLJ = m_sFlockingParams.GeneralizedLennardJones(   fblobDistance,    fTargetDistance, 
                                                                                          fGain,
                                                                                          fExponent);

                     // Check if the robot is within +- 20% of the target distance from neighbors
                     if (  fblobDistance < fTargetDistance * 1.20f && 
                           fblobDistance > fTargetDistance * 0.80f){
                        m_bReachedTargetDistanceFromNeighbors = true;
                     } else {
                        m_bReachedTargetDistanceFromNeighbors = false;
                     }

                     /* Sum to accumulator */
                     cAccum += CVector2(fLJ,sReadings.BlobList[i]->Angle);
                     /* Increment the blobs seen counter */
                     ++unBlobsSeen;

                  } else if (ledColor == CColor::RED){

                     /**
                      * * Currently just Normal Flocking Behavior 
                      * ⚠️ Subject to change. I'm thinking of increasing distance for better wall coverage.
                      */

                     /* Calculate LJ with modified Gain and TargetDistance*/
                     Real fTargetDistance = m_sFlockingParams.TargetDistance;
                     Real fGain = m_sFlockingParams.Gain;
                     Real fExponent = m_sFlockingParams.Exponent;

                     fLJ = m_sFlockingParams.GeneralizedLennardJones(   fblobDistance,    fTargetDistance, 
                                                                                          fGain,
                                                                                          fExponent);

                     // Check if the robot is within +- 20% of the target distance from neighbors
                     if (  fblobDistance < fTargetDistance * 1.20f && 
                           fblobDistance > fTargetDistance * 0.80f){
                        m_bReachedTargetDistanceFromNeighbors = true;
                     } else {
                        m_bReachedTargetDistanceFromNeighbors = false;
                     }

                     /* Sum to accumulator */
                     cAccum += CVector2(fLJ,sReadings.BlobList[i]->Angle);
                     /* Increment the blobs seen counter */
                     ++unBlobsSeen;
                  }
                  break;

               default:
                  //TODO: Do i need an error message here?
                  break;
            }
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

   std::stringstream logstream;

   /* Get the heading angle */
   CRadians cHeadingAngle = c_heading.Angle().SignedNormalize();
   logstream << "cHeadingAngle: " << ToDegrees(cHeadingAngle) << std::endl;
   /* Get the length of the heading vector */
   Real fHeadingLength = c_heading.Length();
   logstream << "fHeadingLength: " << fHeadingLength << std::endl;
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
   logstream << "Turning Mechanism: " << m_sWheelTurningParams.TurningMechanism << std::endl;
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
   if (m_bLoggingEnabled) HFLOG(logstream.str());
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

CRadians CFootBotHydroflock::GetOrientation(){
   const CCI_PositioningSensor::SReading& sReading = m_pcPosition->GetReading();

   CQuaternion q = sReading.Orientation;

   // Declare CRadians for Euler angles
   CRadians cOrientation, cTemp1, cTemp2;

   // Convert quaternion to Euler angles
   q.ToEulerAngles(cOrientation, cTemp1, cTemp2);

   return cOrientation;
}

/****************************************/
/****************************************/

void CFootBotHydroflock::OmniCameraTest(){

   std::stringstream logstream;

   // Ensure m_pcPosition is not null
   if (m_pcPosition != nullptr) {
      // Get the global position and orientation of the receiving robot
      const CCI_PositioningSensor::SReading& tPositionReading = m_pcPosition->GetReading();
      CVector2 cReceiverPosition(tPositionReading.Position.GetX(), tPositionReading.Position.GetY());

      // Print the quaternion values for debugging
      logstream << "Orientation Quaternion: " << tPositionReading.Orientation << std::endl;

      // Declare CRadians for Euler angles
      CRadians cReceiverOrientation, cTemp1, cTemp2;

      // Convert quaternion to Euler angles
      tPositionReading.Orientation.ToEulerAngles(cReceiverOrientation, cTemp1, cTemp2);

      // Get the blobs detected by the camera
      const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& sBlobReadings = m_pcCamera->GetReadings();
      logstream << "Blobs detected: " << sBlobReadings.BlobList.size() << std::endl;

      size_t it = 0;
      // Convert blob positions to global coordinates
      for (const auto& blob : sBlobReadings.BlobList) {
         if (blob->Color == CColor::RED) {
            CRadians adjustedBlobAngle = blob->Angle + cReceiverOrientation;
            CVector2 cBlobPosition(blob->Distance / 100 * Cos(adjustedBlobAngle), blob->Distance / 100 * Sin(adjustedBlobAngle));
            cBlobPosition += cReceiverPosition;
            logstream << "Blob " << it << " Position = " << cBlobPosition << std::endl;
         }
         it++;
      }
   } else {
      argos::LOGERR << "Positioning sensor not initialized." << std::endl;
   }

   if (m_bLoggingEnabled) HFLOG(logstream.str());
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
