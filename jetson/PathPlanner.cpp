#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DriverStation.h>
#include <frc2/command/Subsystem.h>

#include <ntcore_c.h>
#include <ntcore.h>
#include <fmt/core.h>

#include <math.h>

#define M_PI 3.14159265358979323846264338327950288

class PathPlanner {
  public:
    NT_Inst* jetsonServer = 0;
    NT_Subscriber* startVelSubscriber = 0;
    NT_Subscriber* startPosSubscriber = 0;
    NT_Subscriber* finalPosSubscriber = 0;
    NT_Subscriber* finalVelSubscriber = 0;
    NT_Publisher* pathVelocityXPublisher = 0;
    NT_Publisher* pathVelocityYPublisher = 0;
    NT_Publisher* pathVelocityOmegaPublisher = 0;
    NT_Publisher* pathCompletedPublisher = 0;

    double pathVelocityX = 0;  
    double pathVelocityY = 0;  
    double pathVelocityOmega = 0;  

    double startPos[3] = {0};
    double startVel[3] = {0};

    double finalPos[3] = {0};
    double finalVel[3] = {0};

    double adjustedVector[2] = {0};

    //Max velocity of the robot in m/s
    double maxVel = 1;

    //Max acceleration of the robot in m/s^2
    double maxAccel = 1;

    //Max omega of the robot in deg/s
    double maxTurnVel = 90;

    //Max alpha of the robot in deg/s^2
    double maxTurnAccel = 90;

    //Path complete boolean
    bool completed = false;

    PathPlanner() {
      *jetsonServer = nt::CreateInstance();
      nt::SetServer(*jetsonServer, "jetson", NT_DEFAULT_PORT4);
      nt::StartServer(*jetsonServer, "networktables.json", "10.36.92.11", NT_DEFAULT_PORT3, NT_DEFAULT_PORT4);
      *startVelSubscriber = nt::Subscribe(NT_GetTopic(*jetsonServer, "/roborio/swervemaster/startVel", 31), NT_DOUBLE_ARRAY, "double_array");
      *startPosSubscriber = nt::Subscribe(NT_GetTopic(*jetsonServer, "/roborio/swervemaster/startPos", 31), NT_DOUBLE_ARRAY, "double_array");
      *finalPosSubscriber = nt::Subscribe(NT_GetTopic(*jetsonServer, "/roborio/swervemaster/finalPos", 31), NT_DOUBLE_ARRAY, "double_array");
      *finalVelSubscriber = nt::Subscribe(NT_GetTopic(*jetsonServer, "/roborio/swervemaster/finalVel", 31), NT_DOUBLE_ARRAY, "double_array");
      *pathVelocityXPublisher = nt::Publish(NT_GetTopic(*jetsonServer, "/roborio/swervemaster/pathVelocitiesX", 38), NT_DOUBLE, "double");
      *pathVelocityYPublisher = nt::Publish(NT_GetTopic(*jetsonServer, "/roborio/swervemaster/pathVelocitiesY", 38), NT_DOUBLE, "double");
      *pathVelocityOmegaPublisher = nt::Publish(NT_GetTopic(*jetsonServer, "/roborio/swervemaster/pathVelocitiesOmega", 42), NT_DOUBLE, "double");
      *pathCompletedPublisher = nt::Publish(NT_GetTopic(*jetsonServer, "/roborio/swervemaster/pathCompleted", 36), NT_BOOLEAN, "boolean");
    }

    //Get starting position from the network table
    void getStartPos() {
        //Grab start position values from the network table as [x, y, angle]
        std::vector<double> inputVector = nt::GetDoubleArray(*startPosSubscriber, std::span<double>{});
        double positions;
        positions = inputVector.front();
        
        //Create position array using values
        double pose[] = {positions, *(&positions + 1), *(&positions + 2)};

        //Set start positions
        startPos[0] = pose[0];
        startPos[1] = pose[1];
        startPos[2] = pose[2];
    }

    //Get starting speeds from the network table
    void getStartVel() {
        //Grab start velocity values from the network table as [xVel, yVel, turnVel]
        std::vector<double> inputVector = nt::GetDoubleArray(*startVelSubscriber, std::span<double>{});
        double velocities;
        velocities = inputVector.front();

        //Create velocities array using values
        double speeds[] = {velocities, *(&velocities + 1), *(&velocities + 2)};

        //Set start velocity 
        startVel[0] = std::sqrt(std::pow(std::abs(speeds[0]), 2) + std::pow(std::abs(speeds[1]), 2));
        startVel[1] = std::atan2(speeds[1], speeds[0]);
        startVel[2] = speeds[2];
    }

    //Publish path planning velocities
    void publishSpeeds() {
        nt::SetFloat(*pathVelocityXPublisher, pathVelocityX);
        nt::SetFloat(*pathVelocityYPublisher, pathVelocityY);
        nt::SetFloat(*pathVelocityOmegaPublisher, pathVelocityOmega);
        if (std::abs(startPos[0] - finalPos[0]) < 0.01 && std::abs(startPos[1] - finalPos[1]) < 0.01 && std::abs(startPos[2] - finalPos[2]) < 0.05) {
            completed = true;
        }
        nt::SetBoolean(*pathCompletedPublisher, completed);
    }

    void getFinalPos() {
        //Grab final position values from the network table as [x, y, angle]
        std::vector<double> inputVector = nt::GetDoubleArray(*finalPosSubscriber, std::span<double>{});
        double positions;
        positions = inputVector.front();

        //Create position array using values
        double pose[] = {positions, *(&positions + 1), *(&positions + 2)};

        //Set final positions 
        if (finalPos[0] != pose[0] || finalPos[1] != pose[1] || finalPos[2] != pose[2]) {
            completed = false;
        }
        finalPos[0] = pose[0];
        finalPos[1] = pose[1];
        finalPos[2] = pose[2];
    }

    void getFinalVel() {
        //Grab final velocity values from the network table as [xVel, yVel, turnVel]
        std::vector<double> inputVector = nt::GetDoubleArray(*finalVelSubscriber, std::span<double>{});
        double velocities;
        velocities = inputVector.front();

        //Create velocity array using values
        double speeds[] = {velocities, *(&velocities + 1), *(&velocities + 2)};

        //Set final velocity
        finalVel[0] = std::sqrt(std::pow(std::abs(speeds[0]), 2) + std::pow(std::abs(speeds[1]), 2));
        finalVel[1] = std::atan2(speeds[1], speeds[0]);
        finalVel[2] = speeds[2];
    }

    //Calculate optimal velocities at current position to get to final position
    void calculateVelocities() {
        //Grab start values
        getStartPos();
        getStartVel();

        //Grab final values
        getFinalPos();
        getFinalVel();

        //Create optimal velocity vector 
        double velocityMagnitude = startVel[0];
        double velocityAngle = std::atan2(startPos[1] - finalPos[1], startPos[0] - finalPos[0]) + M_PI;

        
        double adjustedFinalAngle = finalVel[1] - M_PI; //Do a 180 on the final vector
        if (adjustedFinalAngle < 0) {
            adjustedFinalAngle += 2 * M_PI;
        }
        calculateAdjustedVector(velocityMagnitude, velocityAngle, finalVel[0], adjustedFinalAngle, startVel[0], startVel[1], 1);
        velocityMagnitude = adjustedVector[0];
        velocityAngle = adjustedVector[1];
        
        if (velocityAngle > 2 * M_PI) {
            velocityAngle -= 2 * M_PI;
        } 

        if (velocityAngle < 0) {
            velocityAngle += 2 * M_PI;
        } 

        //Calculate total distance to move
        double distance = std::sqrt(std::pow(std::abs(startPos[0] - finalPos[0]), 2) + std::pow(std::abs(startPos[1] - finalPos[1]), 2));

        //Find position change to deccelerate from start velocity to end velocity
        double timeDeccel = (startVel[0] - finalVel[0]) / maxAccel;
        double deltaDeccelPos = (finalVel[0] + startVel[0]) * timeDeccel / 2;

        //Figure out if we accelerate or deccelerate
        if (distance < deltaDeccelPos) { //If distance covered during decceleration is too large, Deccelerate
            velocityMagnitude -= maxAccel / 50.0;
            if (velocityMagnitude < 0) {
                velocityMagnitude = 0;
            }
        } else { //Else, Accelerate
            velocityMagnitude += maxAccel / 50.0;
            if (velocityMagnitude > maxVel) { //Max out velocity
                velocityMagnitude = maxVel;
            }
        }

        //Calculate the x and y velocities using the angle and magnitude.
        pathVelocityX = std::cos(velocityAngle) * velocityMagnitude;
        pathVelocityY = std::sin(velocityAngle) * velocityMagnitude;

        //Repeat with omega
        //Create optimal omega 
        double omega = startVel[2];

        //Calculate total distance to turn
        double angle = finalPos[2] - startPos[2]; 

        //Find angle change to deccelerate from start omega to end omega
        timeDeccel = (startVel[2] - finalVel[2]) / maxTurnAccel;
        double deltaDeccelAngle = (finalVel[2] + startVel[2]) * timeDeccel / 2;

        //Figure out if we accelerate or deccelerate
        if (startPos[2] >= finalPos[2] - 0.005) { //Tolerance in meters
            omega = 0;
        } else if (angle < deltaDeccelAngle) { //If angle covered during decceleration is too large, Deccelerate
            omega -= maxTurnAccel / 50.0;
        } else { //Else, Accelerate
            omega += maxTurnAccel / 50.0;
            if (omega > maxTurnVel) { //Max out velocity
                omega = maxTurnVel;
            }
        }

        //Set path turn velocity to omega
        pathVelocityOmega = omega;

        //Publish new speeds
        publishSpeeds();
    }

    //Given the current, end, and starting vectors add them with a factor based on how far away 
    // the starting vector angle is from the ending vector angle
    void calculateAdjustedVector(double vel1, double ang1, double vel2, double ang2, double vel3, double ang3, double weight) {
        double difference = std::fmin(std::abs(ang2 - ang3), std::fmin(std::abs(ang2 - ang3 - M_PI * 2), std::abs(ang2 - ang3 + M_PI * 2)));
        double factor = difference / M_PI * 2;
        if (vel1 != 0) {
        vel2 *= (2 - factor) * vel1 * 0.5;    
        vel3 *= factor * vel1 * 0.5;
        }
        double vel1X = std::cos(ang1) * vel1;
        double vel1Y = std::sin(ang1) * vel1;
        double vel2X = std::cos(ang2) * vel2;
        double vel2Y = std::sin(ang2) * vel2;
        double vel3X = std::cos(ang3) * vel3;
        double vel3Y = std::sin(ang3) * vel3;
        double finalVelX = vel1X + vel2X + vel3X;
        double finalVelY = vel1Y + vel2Y + vel3Y;
        adjustedVector[1] = std::atan2(finalVelY, finalVelX);
        adjustedVector[0] = vel1;
    }
};