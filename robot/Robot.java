// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  //The PS4Controller for driving
  private PS4Controller driveController;

  //The current factor to multiply driveController inputs by -> 0, 0.25, 0.5, 0.75, or 1 (basically different speed levels)
  public static double driveControllerFactor;

  //Turn factor
  public static double turnFactor;

  //Self-explanatory
  private SwerveMaster mySwerveMaster;
  
  @Override
  public void robotInit() {
    SmartDashboard.putString("Current mode: ", "robotInit");

    m_robotContainer = new RobotContainer();

    //Controller variables
    driveController = new PS4Controller(Constants.driveControllerPort);
    driveControllerFactor = 0.25d;
    turnFactor = 0.25d;

    //Create SwerveMaster object
    mySwerveMaster = new SwerveMaster();

    //Set mode to brake
    mySwerveMaster.leftUpModule.driveMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    mySwerveMaster.leftDownModule.driveMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    mySwerveMaster.rightUpModule.driveMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    mySwerveMaster.rightDownModule.driveMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

    //Reset accelerometer on startup
    mySwerveMaster.resetAccelerometer();
  }

  //Every 20ms
  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putString("Mode: ", "teleopPeriodic");

    if(driveController.getTouchpadPressed()) {
      driveControllerFactor = 0d;
    } else if(driveController.getSquareButtonPressed()) {
      driveControllerFactor = 0.25d;
    } else if(driveController.getCrossButtonPressed()) {
      driveControllerFactor = 0.5d;
    } else if(driveController.getCircleButtonPressed()) {
      driveControllerFactor = 0.75d;
    } else if(driveController.getTriangleButtonPressed()) {
      driveControllerFactor = 1.0d;
    }

    if(driveController.getPSButtonPressed()) {
      turnFactor = 0d;
    } else if(driveController.getPOV() == 270) {
      turnFactor = 0.25d;
    } else if(driveController.getPOV() == 180) {
      turnFactor = 0.5d;
    } else if(driveController.getPOV() == 90) {
      turnFactor = 0.75d;
    } else if(driveController.getPOV() == 0) {
      turnFactor = 1.0d;
    }

    //Accelerometer will need to be reset due to inaccuracies accumulating
    //Odometry - Resets the origin and angle to the current position and angle of the robot
    if (driveController.getOptionsButtonPressed()) {
      mySwerveMaster.resetOrigin();
    }

    //Controller Smart Dashboard values
    SmartDashboard.putNumber("Drive Factor: ", driveControllerFactor);
    SmartDashboard.putNumber("Turn Factor: ", turnFactor);
    SmartDashboard.putNumber("DC Left X: ", driveController.getLeftX());    
    SmartDashboard.putNumber("DC Left Y: ", driveController.getLeftY());
    SmartDashboard.putNumber("DC Right X: ", driveController.getRightX());

    //Call main method for swerve drive
    mySwerveMaster.update(driveController, driveControllerFactor, turnFactor);
  }

  @Override
  public void disabledInit() {
    SmartDashboard.putString("Mode: ", "disabledInit");

    //Tell motors to stop
    mySwerveMaster.set(new double[]{0d, 0d, 0d, 0d}, new double[]{0d, 0d, 0d, 0d});
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
