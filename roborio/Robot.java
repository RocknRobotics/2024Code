// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  //The PS4Controller for driving
  private PS4Controller driveController;

  //The current factor to multiply driveController inputs by -> 0, 0.25, 0.5, 0.75, or 1 (basically different speed levels)
  public static double driveControllerFactor;

  //Turn factor
  public static double turnFactor;

  //Self-explanatory
  private SwerveMaster mySwerveMaster;

  private HookMaster myHookMaster;
  private ArmMaster myArmMaster;

  private PS4Controller armController;

  public static double fireFactor;
  public static double angleFactor;
    
  @Override
  public void robotInit() {
    NetworkTableInstance inst = NetworkTableInstance.createInstance();
    inst.startServer();
    SmartDashboard.setNetworkTableInstance(inst);
    SmartDashboard.putString("Current mode: ", "robotInit");

    //Controller variables
    driveController = new PS4Controller(Constants.driveControllerPort);
    driveControllerFactor = 0.25d;
    turnFactor = 0.25d;

    //Create SwerveMaster object
    mySwerveMaster = new SwerveMaster();

    //Reset accelerometer on startup
    mySwerveMaster.resetAccelerometer();

    myHookMaster = new HookMaster();
    myArmMaster = new ArmMaster();

    armController = new PS4Controller(Constants.armControllerPort);
    fireFactor = 0.25;
    angleFactor = 0.25;

    SmartDashboard.putString("Auto-Shoot Status: ", "DON'T FIRE YET");
  }

  //Every 20ms
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Arm Angle: ", myArmMaster.getAngle());
    SmartDashboard.putNumber("LU Angle: ", mySwerveMaster.leftUpModule.getAbsoluteTurnPosition());
    SmartDashboard.putNumber("LD Angle: ", mySwerveMaster.leftDownModule.getAbsoluteTurnPosition());
    SmartDashboard.putNumber("RU Angle: ", mySwerveMaster.rightUpModule.getAbsoluteTurnPosition());
    SmartDashboard.putNumber("RD Angle: ", mySwerveMaster.rightDownModule.getAbsoluteTurnPosition());
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    if(driveController.getTouchpadPressed()) {
      driveControllerFactor = 0d;
    } else if(driveController.getL2Button()) {
      driveControllerFactor = Math.min(1, driveControllerFactor + 0.025);
    } else if(driveController.getL1Button()) {
      driveControllerFactor = Math.max(0.1, driveControllerFactor - 0.025);
    }

    if(driveController.getPSButtonPressed()) {
      turnFactor = 0d;
    } else if(driveController.getR2Button()) {
      turnFactor = Math.min(1, turnFactor + 0.025);
    } else if(driveController.getR1Button()) {
      turnFactor = Math.max(0.1, turnFactor - 0.025);
    }

    if(armController.getTouchpadPressed()) {
      angleFactor = 0d;
    } else if(armController.getL2Button()) {
      angleFactor = Math.min(1, angleFactor + 0.025);
    } else if(armController.getL1Button()) {
      angleFactor = Math.max(0.1, angleFactor - 0.025);
    }

    if(armController.getPSButtonPressed()) {
      fireFactor = 0d;
    } else if(armController.getR2Button()) {
      fireFactor = Math.min(1, fireFactor + 0.025);
    } else if(armController.getR1Button()) {
      fireFactor = Math.max(0.1, fireFactor - 0.025);
    }

    //Accelerometer will need to be reset due to inaccuracies accumulating
    //Odometry - Resets the origin and angle to the current position and angle of the robot
    if (driveController.getShareButtonPressed()) {
      mySwerveMaster.resetPose(0, 0, 0);
    }

    myHookMaster.update(armController.getShareButtonPressed());
    myArmMaster.update(armController, mySwerveMaster.getRobotPosition()[0], mySwerveMaster.getRobotPosition()[1]);

    //Controller Smart Dashboard values
    SmartDashboard.putNumber("Drive Factor: ", driveControllerFactor);
    SmartDashboard.putNumber("Turn Factor: ", turnFactor);
    SmartDashboard.putNumber("DC Left X: ", driveController.getLeftX());    
    SmartDashboard.putNumber("DC Left Y: ", driveController.getLeftY());
    SmartDashboard.putNumber("DC Right X: ", driveController.getRightX());

    SmartDashboard.putNumber("Angle Factor: ", angleFactor);
    SmartDashboard.putNumber("Fire Factor: ", fireFactor);
    SmartDashboard.putNumber("AC Left Y: ", armController.getLeftY());    
    SmartDashboard.putNumber("AC Right Y: ", armController.getRightY());

    //Call main method for swerve drive
    mySwerveMaster.update(driveController, driveControllerFactor, turnFactor);

    if(SmartDashboard.getString("Launcher Status: ", "NOT YET").equals("LAUNCHER READY") && 
      SmartDashboard.getString("Heading Status: ", "NOT YET").equals("HEADING READY") && 
      SmartDashboard.getString("Arm Angle Status: ", "NOT YET").equals("ARM ANGLE READY") && 
      SmartDashboard.getString("Distance Range Status: ", "NOT YET").equals("DISTANCE RANGE READY") && 
      SmartDashboard.getString("Angle Range Status: ", "NOT YET").equals("ANGLE RANGE READY")) {
        SmartDashboard.putString("Auto-Shoot Status: ", "ALL SYSTEMS GO!!!");
      } else {
        SmartDashboard.putString("Auto-Shoot Status: ", "DON'T FIRE YET");
      }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    //Since we added accel rates, we need to continuously bring them to 0 in case they didn't get there in one tick
    mySwerveMaster.stop();;
    myHookMaster.stop();
    myArmMaster.stop();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
    /*mySwerveMaster.leftUpModule.driveMotor.set(driveController.getLeftY());
    mySwerveMaster.leftDownModule.driveMotor.set(driveController.getRightY());
    mySwerveMaster.rightUpModule.driveMotor.set(driveController.getLeftY());
    mySwerveMaster.rightDownModule.driveMotor.set(driveController.getRightY());*/
    /*mySwerveMaster.leftUpModule.turnMotor.set(driveController.getLeftY());
    mySwerveMaster.leftDownModule.turnMotor.set(driveController.getRightY());
    mySwerveMaster.rightUpModule.turnMotor.set(driveController.getLeftY());
    mySwerveMaster.rightDownModule.turnMotor.set(driveController.getRightY());*/
    /*myArmMaster.launcherAngle.set(driveController.getLeftY());
    myArmMaster.myIntake.groundRoller.set(driveController.getRightY());*/
    /*myArmMaster.myIntake.bottomIntake.set(driveController.getLeftY());
    myArmMaster.myIntake.topIntake.set(driveController.getRightY());*/
    /*myArmMaster.myLauncher.backMiddleRoller.set(driveController.getLeftY());
    myArmMaster.myLauncher.frontMiddleRoller.set(driveController.getRightY());*/
    /*myArmMaster.myLauncher.backLauncher.set(driveController.getLeftY());
    myArmMaster.myLauncher.frontLauncher.set(driveController.getRightY());*/
    mySwerveMaster.leftDownModule.driveMotor.set(driveController.getLeftY());

    SmartDashboard.putNumber("Drive Controller Left Y: ", driveController.getLeftY());
    SmartDashboard.putNumber("Drive Controller Right Y: ", driveController.getRightY());
  }
}