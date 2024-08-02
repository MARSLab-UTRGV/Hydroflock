


/**
 * Considering using the distance scanner (LiDAR) instaed of proximity sensor
 * 
 * Need to manage states and transition between states
 * 
 * Filtering out proximity readings that correlate with the omni-cam readings (other bots)
 * 
 * How to know when a corner has been reached?
 * 
 * Should I have a time buffer before switching to Aggregator state? Need to be sure that the vector is unobstructed.
 * 
 * How to do aggregation? Aggregator needs to wait for aggregatee to get close. Cannot loose sight/connection of aggregator
 * 
 */












#ifndef FOOTBOT_HYDROFLOCK_H
#define FOOTBOT_HYDROFLOCK_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the omnidirectional camera sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
/* Definition of the foot-bot light sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
/* Vector2 definitions */
#include <argos3/core/utility/math/vector2.h>
/* Definition of the positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the range and bearing actuator*/
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>

#include <rab_dsr.h>
#include <set>
#include <unordered_map>
#include <algorithm>

using namespace argos;

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sys/stat.h>
#include <unistd.h>
#include <fstream>

#define LOG(str) LogThis(str, __FUNCTION__)


class CFootBotHydroflock : public CCI_Controller {

public:

   /**
    * The following variables are used as parameters for
    * turning during navigation. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><footbot_hydroflock_controller><parameters><wheel_turning>
    * section.
    */
   struct SWheelTurningParams {
      /**
       * The turning mechanism.
       * The robot can be in three different turning states.
       */
      enum ETurningMechanism
      {
         NO_TURN = 0, // go straight
         SOFT_TURN,   // both wheels are turning forwards, but at different speeds
         HARD_TURN    // wheels are turning with opposite speeds
      } TurningMechanism;
      /**
       * Angular thresholds to change turning state.
       */
      CRadians HardTurnOnAngleThreshold;
      CRadians SoftTurnOnAngleThreshold;
      CRadians NoTurnAngleThreshold;
      /* Maximum wheel speed */
      Real MaxSpeed;

      void Init(TConfigurationNode& t_tree);
   };

   /**
    * The following variables are used as parameters for
    * flocking interaction. You can set their value
    * in the <parameters> section of the XML configuration
    * file, under the
    * <controllers><footbot_hydroflock_controller><parameters><flocking>
    * section.
    */
   struct SFlockingInteractionParams {
      /* Target robot-robot distance in cm */
      Real TargetDistance;
      /* Gain of the Lennard-Jones potential */
      Real Gain;
      /* Exponent of the Lennard-Jones potential */
      Real Exponent;
      /* GREEN -> RED Interaction Gain Modifier */
      Real GR_GMod;
      /* GREEN -> RED Interaction TargetDistance Modifier */
      Real GR_TDMod;
      /* RED -> GREEN Interaction Gain Modifier */
      Real RG_GMod;
      /* RED -> GREEN Interaction TargetDistance Modifier */
      Real RG_TDMod;
      /* RED -> RED Interaction Gain Modifier */
      Real RR_GMod;
      /* RED -> RED Interaction TargetDistance Modifier */
      Real RR_TDMod;

      void Init(TConfigurationNode& t_node);
      Real GeneralizedLennardJones( const Real& f_current_distance, const Real& f_target_distance, 
                                    const Real& f_gain, const Real& f_exponent);
   };


   enum FlockingState {
      DEFAULT,             // Normal flocking with no modifications to Lennard-Jones potential
      WALL_DISPERSION,     // Flocking with wall dispersion
      WALL_FOLLOWING,      // Flocking with wall following (after max distance from all neighbors are reached in WALL_DISPERSION)
      AGGREGATOR,          // The aggregator has found away around the wall
      AGGREGATEE           // The aggregatee has not found a way but will aggregate toward the AGGREGATORS
   } m_eFState = DEFAULT;


public:

   /* Class constructor. */
   CFootBotHydroflock();

   /* Class destructor. */
   virtual ~CFootBotHydroflock() {}

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML file
    * in the <controllers><footbot_flocking_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything, so
    * the function could have been omitted. It's here just for completeness.
    */
   virtual void Reset();

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up, so
    * the function could have been omitted. It's here just for completeness.
    */
   virtual void Destroy();

   CVector2 GetTargetLocation() { return m_cTargetPosition; };

protected:

   /*
    * Calculates the vector to the closest light.
    */
   virtual CVector2 VectorToLight();

   /*
    * Calculates the flocking interaction vector.
    */
   virtual CVector2 FlockingVector();

   /*
    * Gets a direction vector as input and transforms it into wheel actuation.
    */
   void SetWheelSpeedsFromVector(const CVector2& c_heading);

   /**
    * Search pattern helper function to get the robot ID
    */
   virtual size_t GetRobotIndex();

   /**
    * Search pattern helper function to get current position
    */
   virtual CVector2 GetCurrentPosition();

   CRadians GetOrientation();

   CVector2 VectorToTarget();

   void SetFlockingState(const FlockingState& f_state);

   void StateUpdater();

   void GetNeighborStates();

   /**
    * Test function for the omni-cam
    */
   void OmniCameraTest();

   bool DetectWall();

   bool TargetVectorUnobstructed();

   CVector2 CalculateWallRepulsion();

   CVector2 CalculateTangentialMovement(const CVector2& f_cFlockingVector);

   CVector2 VectorToWall();

   CVector2 ReorientTowardsWall();



   void NormalFlocking();

   void WallDispersion();

   void WallFollowing();

   void Aggregator();

   void Aggregatee();



   std::vector<size_t> GetRelevantProximitySensors(const CVector2& f_cTargetVector, const Real& f_fCustomThreshold = -1.0f);

   bool RobotInProximity(const CRadians& f_cProximityAngle);

   bool OuterCornerDetected();

   bool InnerCornerDetected();

   bool CornerDetected();

   /**
    * @brief Maps vector `f_cVectorA` onto vector `f_cVectorB`. When you project a vector
    * onto another vector, you are essentially finding out how much of the first vector
    * lies along the direction of the second vector.
    * 
    * @param f_cVectorA 
    * @param f_cVectorB 
    * @return The resulting vector after projection. 
    */
   CVector2 ProjectVectorOnVector(const CVector2& f_cVectorA, const CVector2& f_cVectorB);

private:

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the foot-bot light sensor */
   CCI_FootBotLightSensor* m_pcLight;
   /* Pointer to the LEDs actuator */
   CCI_LEDsActuator* m_pcLEDs;
   /* Pointer to the omnidirectional camera sensor */
   CCI_ColoredBlobOmnidirectionalCameraSensor* m_pcCamera;
   /* Pointer to the positioning sensor */
   CCI_PositioningSensor* m_pcPosition;
   /* Pointer to the range and bearing sensor */
   CCI_RangeAndBearingSensor* m_pcRABSens;
   /* Pointer to the range and bearing actuator */
   CCI_RangeAndBearingActuator* m_pcRABActuator;
   /* Pointer to the proximity sensor */
   CCI_FootBotProximitySensor* m_pcProximity;


   /* Boolean used to keep robots in place for communication testing */
   bool m_bCommunicationTest;
   /* Last vector to wall */
   CVector2 m_cLastVectorToWall;
   /* Last known position when in contact with wall */
   CVector2 m_cLastWallContactPosition;
   /* Last known heading when in contact with wall */
   CRadians m_cLastWallContactHeading;
   /* Angle threshold for proximity sensor readings */
   CDegrees m_cAlpha;

   bool m_bPrintState;

   /* Boolean set when maximum distance from neighbors is reached */
   bool m_bReachedTargetDistanceFromNeighbors;

   /* The target location */
   CVector2 m_cTargetPosition;

   /* The turning parameters. */
   SWheelTurningParams m_sWheelTurningParams;
   /* The flocking interaction parameters. */
   SFlockingInteractionParams m_sFlockingParams;

   CVector2 m_cLastVectorToTarget;
   /* Simulation time step. */
   Real TimeStep;
   /* Acceptable range for correlating positions from RAB and omni-cam sensors. */
   Real m_fAcceptableRange;

   bool m_bHasGreenNeighbor, m_bHasRedNeighbor, m_bHasPurpleNeighbor, m_bHasBlueNeighbor, m_bHasMagentaNeighbor;

   /* Variable to count simulation ticks */
   UInt32 m_unTicks;
   
   CDynamicSourceRouting m_cRab_Dsr;

   /* Vector of previous proximity readings used for corner detection */
   std::vector<Real> m_vecPreviousProximityReadings;


   /**
    * * Log File Stuff
    */
   bool m_bLoggingEnabled;
   std::ofstream m_Log;
   size_t m_fLogFrequency;
   const std::string m_sLogPath = "./controllers/footbot_hydroflock/controller_logs/";
   std::string m_sLogFileExt = ".log";
   std::string m_sLogFilePath;
   void LogInit();
   void LogThis(const std::string& f_sMessage, const std::string& f_sFunctionName);


};

#endif
