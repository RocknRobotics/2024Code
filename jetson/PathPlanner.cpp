#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DriverStation.h>
#include <frc2/command/Subsystem.h>

#include <ntcore_c.h>
#include <ntcore.h>
#include <fmt/core.h>

using namespace pathplanner;

class PathPlanner: public frc2::Subsystem {
  public:
    NT_Inst* jetsonServer = 0;
    NT_Subscriber* chassisSubscriber = 0;
    NT_Subscriber* poseSubscriber = 0;
    NT_Publisher* resetPoseArrayPublisher = 0;
    NT_Publisher* resetPoseBooleanPublisher = 0;
    NT_Publisher* setChassisSpeedsArrayPublisher = 0;


    PathPlanner() {
      *jetsonServer = nt::CreateInstance();
      nt::SetServer(*jetsonServer, "jetson", NT_DEFAULT_PORT4);
      nt::StartServer(*jetsonServer, "networktables.json", "10.36.92.11", NT_DEFAULT_PORT3, NT_DEFAULT_PORT4);
      *chassisSubscriber = nt::Subscribe(NT_GetTopic(*jetsonServer, "/roborio/swervemaster/chassis", 30), NT_DOUBLE_ARRAY, "double_array");
      *poseSubscriber = nt::Subscribe(NT_GetTopic(*jetsonServer, "/roborio/swervemaster/pose", 27), NT_DOUBLE_ARRAY, "double_array");
      *resetPoseArrayPublisher = nt::Publish(NT_GetTopic(*jetsonServer, "/roborio/swervemaster/resetPoseArray", 37), NT_DOUBLE_ARRAY, "double_array");
      *resetPoseBooleanPublisher = nt::Publish(NT_GetTopic(*jetsonServer, "/roborio/swervemaster/resetPoseBoolean", 39), NT_BOOLEAN, "boolean");
      *setChassisSpeedsArrayPublisher = nt::Publish(NT_GetTopic(*jetsonServer, "/roborio/swervemaster/setChassisSpeeds", 39), NT_DOUBLE_ARRAY, "double_array");
    }

    void startUp() {
      // Configure the AutoBuilder last
      AutoBuilder::configureHolonomic(
        // Robot pose supplier
        [this](){ return getPose2d(); },

        // Method to reset odometry (will be called if your auto has a starting pose)
        [this](frc::Pose2d pose){ resetPose2d(pose); },

        // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](){ return getChassisSpeeds(); }, 

        // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        [this](frc::ChassisSpeeds speeds){ setChassisSpeeds(speeds); },

        //Inputs(Translation PID, Rotation PID, Max motor speed, Radius of robot, Default path replanning config)
        HolonomicPathFollowerConfig(PIDConstants(5.0, 0.0, 0.0), PIDConstants(5.0, 0.0, 0.0), 1_mps, 0.969_m, ReplanningConfig()),
        []() {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          auto alliance = frc::DriverStation::GetAlliance();
          if (alliance) {
            return alliance.value() == frc::DriverStation::Alliance::kRed;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
      );
    }

    //Get robot Pose2d from the network table
    frc::Pose2d getPose2d() {
      //Grab pose values from the network table
      std::vector<double> inputVector = nt::GetDoubleArray(*poseSubscriber, std::span<double>{});
      double poseVals;
      poseVals = inputVector.front();
      
      //Create Pose2d using values
      frc::Pose2d pose(units::unit_t<units::meters>{poseVals}, units::unit_t<units::meters>{*(&poseVals + 1)}, frc::Rotation2d(units::radian_t{*(&poseVals + 2)}));

      //Return pose
      return pose;
    }

    //Get Chassis Speeds from the network table
    frc::ChassisSpeeds getChassisSpeeds() {
      //Grab Chassis values from the network table
      std::vector<double> inputVector = nt::GetDoubleArray(*chassisSubscriber, std::span<double>{});
      double chassisVals;
      chassisVals = inputVector.front();
      
      //Create Pose2d using values
      frc::ChassisSpeeds speeds;
      speeds.vx = units::unit_t<units::compound_unit<units::meters, units::inverse<units::seconds>>>{chassisVals};
      speeds.vy = units::unit_t<units::compound_unit<units::meters, units::inverse<units::seconds>>>{*(&chassisVals + 1)};
      speeds.omega = units::unit_t<units::compound_unit<units::radians, units::inverse<units::seconds>>>{*(&chassisVals + 2)};

      //Return speeds 
      return speeds;
    }

    //Send pose to robot to set as new pose
    void resetPose2d(frc::Pose2d pose) {
      
    }

    //Set the ROBOT RELATIVE Chassis Speeds for the path on the network table
    void setChassisSpeeds(frc::ChassisSpeeds speeds) {

    }
};