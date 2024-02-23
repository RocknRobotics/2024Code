#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DriverStation.h>
#include <frc2/command/Subsystem.h>

#include <ntcore_c.h>
#include <ntcore.h>
#include <fmt/core.h>

#include <math.h>

class PathPlanner {
  public:
    NT_Inst* jetsonServer = 0;
    NT_Subscriber* chassisSubscriber = 0;
    NT_Subscriber* poseSubscriber = 0;
    NT_Subscriber* finalPosSubscriber = 0;
    NT_Subscriber* finalVelSubscriber = 0;
    NT_Publisher* pathVelocityXPublisher = 0;
    NT_Publisher* pathVelocityYPublisher = 0;
    NT_Publisher* pathVelocityOmegaPublisher = 0;

    double pathVelocityX = 0;  
    double pathVelocityY = 0;  
    double pathVelocityOmega = 0;  

    double poseArray[3] = {0};
    double speedsArray[3] = {0};

    double finalPos[3] = {0};
    double finalVel[3] = {0};

    PathPlanner() {
      *jetsonServer = nt::CreateInstance();
      nt::SetServer(*jetsonServer, "jetson", NT_DEFAULT_PORT4);
      nt::StartServer(*jetsonServer, "networktables.json", "10.36.92.11", NT_DEFAULT_PORT3, NT_DEFAULT_PORT4);
      *chassisSubscriber = nt::Subscribe(NT_GetTopic(*jetsonServer, "/roborio/swervemaster/chassis", 30), NT_DOUBLE_ARRAY, "double_array");
      *poseSubscriber = nt::Subscribe(NT_GetTopic(*jetsonServer, "/roborio/swervemaster/pose", 27), NT_DOUBLE_ARRAY, "double_array");
      *finalPosSubscriber = nt::Subscribe(NT_GetTopic(*jetsonServer, "/roborio/swervemaster/finalPos", 31), NT_DOUBLE_ARRAY, "double_array");
      *finalVelSubscriber = nt::Subscribe(NT_GetTopic(*jetsonServer, "/roborio/swervemaster/finalVel", 31), NT_DOUBLE_ARRAY, "double_array");
      *pathVelocityXPublisher = nt::Publish(NT_GetTopic(*jetsonServer, "/roborio/swervemaster/pathVelocitiesX", 38), NT_DOUBLE, "double");
      *pathVelocityYPublisher = nt::Publish(NT_GetTopic(*jetsonServer, "/roborio/swervemaster/pathVelocitiesY", 38), NT_DOUBLE, "double");
      *pathVelocityOmegaPublisher = nt::Publish(NT_GetTopic(*jetsonServer, "/roborio/swervemaster/pathVelocitiesOmega", 42), NT_DOUBLE, "double");
    }

    //Get starting position from the network table
    void getStartPos() {
        //Grab start position values from the network table
      std::vector<double> inputVector = nt::GetDoubleArray(*poseSubscriber, std::span<double>{});
      double poseVals;
      poseVals = inputVector.front();
      
      //Create position array using values
      double pose[] = {poseVals, *(&poseVals + 1), *(&poseVals + 2)};

      //Set start positions
      poseArray[0] = pose[0];
      poseArray[1] = pose[1];
      poseArray[2] = pose[2];
    }

    //Get starting speeds from the network table
    void getStartVel() {
      //Grab start velocity values from the network table
      std::vector<double> inputVector = nt::GetDoubleArray(*chassisSubscriber, std::span<double>{});
      double chassisVals;
      chassisVals = inputVector.front();

      //Create velocites array using values
      double speeds[] = {chassisVals, *(&chassisVals + 1), *(&chassisVals + 2)};

      //Set start velocities 
      speedsArray[0] = speeds[0];
      speedsArray[1] = speeds[1];
      speedsArray[2] = speeds[2];
    }

    //Publish path planning velocities
    void publishSpeeds() {
      nt::SetFloat(*pathVelocityXPublisher, pathVelocityX);
      nt::SetFloat(*pathVelocityYPublisher, pathVelocityY);
      nt::SetFloat(*pathVelocityOmegaPublisher, pathVelocityOmega);
    }

    void getFinalPos() {
      //Grab final position values from the network table
      std::vector<double> inputVector = nt::GetDoubleArray(*finalPosSubscriber, std::span<double>{});
      double posVals;
      posVals = inputVector.front();

      //Create position array using values
      double position[] = {posVals, *(&posVals + 1), *(&posVals + 2)};

      //Set final positions 
      finalPos[0] = position[0];
      finalPos[1] = position[1];
      finalPos[2] = position[2];
    }

    void getFinalVel() {
      //Grab final velocity values from the network table
      std::vector<double> inputVector = nt::GetDoubleArray(*finalVelSubscriber, std::span<double>{});
      double velVals;
      velVals = inputVector.front();

      //Create velocity array using values
      double velocity[] = {velVals, *(&velVals + 1), *(&velVals + 2)};

      //Set final velocities 
      finalVel[0] = velocity[0];
      finalVel[1] = velocity[1];
      finalVel[2] = velocity[2];
    }

    //Given starting pose and chassis speeds, [initialX, initialY, angle] and [initialDeltaX, initialDeltaY, omega],
    //  and ending pose and chassis speeds, [finalX, finalY, angle] and [finalDeltaX, finalDeltaY, omega],
    //  calculate velocity x, y, and turn that the robot should follow

    //TODO - Implement a path finding system to avoid given obstacles

    //Parameters: (finalPos as [finalX in meters, finalY in meters, angle in degrees], finalVel as [finalDeltaX in m/s, finalDeltaY in m/s, omega as deg/s])
    void calculateVelocities() {
      //Grab start values
      getStartPos();
      getStartVel();

      //Grab final values
      getFinalPos();
      getFinalVel();

      publishSpeeds();
    }
};