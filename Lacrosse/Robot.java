// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Arm.ArmMaster;
import frc.robot.Hook.HookMaster;
import frc.robot.Swerve.SwerveMaster;
import frc.robot.Values.Determinables;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static NetworkTableInstance rioNetworkClient;

  public static ArmMaster myArmMaster;
  public static SwerveMaster mySwerveMaster;
  public static HookMaster myHookMaster;

  public static double swerveTranslationalFactor;
  public static double swerveRotationalFactor;
  public static double armAngleFactor;
  public static double armFireFactor;

  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  @Override
  public void robotInit() {
    rioNetworkClient = NetworkTableInstance.getDefault();
    rioNetworkClient.startClient4("rioClient");
    rioNetworkClient.setServer("10.36.92.2", NetworkTableInstance.kDefaultPort4);
    SmartDashboard.setNetworkTableInstance(rioNetworkClient);

    Determinables.construct();

    myArmMaster = new ArmMaster(rioNetworkClient);
    mySwerveMaster = new SwerveMaster(rioNetworkClient);
    myHookMaster = new HookMaster(rioNetworkClient);

    swerveTranslationalFactor = 0.75;
    swerveTranslationalFactor = 0.75;
    armAngleFactor = 0.75;
    armFireFactor = 0.4;
  }

  //20 ms
  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  @Override
  public void robotPeriodic() {}

  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  @Override
  public void autonomousInit() {
    //Falls back to blue if there is no valid alliance or the DS is not connected
    Determinables.construct();
  }

  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  @Override
  public void autonomousPeriodic() {
  }

  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  @Override
  public void teleopInit() {}

  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  @Override
  public void teleopPeriodic() {
    mySwerveMaster.teleopUpdate();
    myArmMaster.teleopUpdate();
    myHookMaster.teleopUpdate();
  }

  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  @Override
  public void disabledInit() {}

  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  @Override
  public void disabledPeriodic() {
    myArmMaster.stop();
    mySwerveMaster.stop();
    myHookMaster.stop();
  }

  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  @Override
  public void testInit() {}

  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  //-----------------------------------------------------------------------------------------------------------------------------------
  @Override
  public void testPeriodic() {}

}