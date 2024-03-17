// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
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

  //private HookMaster myHookMaster;
  private ArmMaster myArmMaster;

  private PS4Controller armController;

  public static double fireFactor;
  public static double angleFactor;

  public CameraMaster myCameraMaster;

  public int autoNum;

  //public Relay testAngleMotor;
    
  @Override
  public void robotInit() {
    //SmartDashboard.putString("Current mode: ", "robotInit");
    //SmartDashboard.putBoolean("Run Camera Stuff", true);

    //Controller variables
    driveController = new PS4Controller(Constants.driveControllerPort);
    driveControllerFactor = 0.75d;
    turnFactor = 0.75d;

    //Create SwerveMaster object
    mySwerveMaster = new SwerveMaster();

    //Reset accelerometer on startup
    mySwerveMaster.resetAccelerometer();

    //myHookMaster = new HookMaster();
    myArmMaster = new ArmMaster();

    armController = new PS4Controller(Constants.armControllerPort);
    fireFactor = 0.645d;
    angleFactor = 0.5d;

    SmartDashboard.putString("Auto-Shoot Status: ", "DON'T FIRE YET");

    //myCameraMaster = new CameraMaster();

    /*Thread cameraThread = new Thread(() -> {
      while(true) {
        if(SmartDashboard.getBoolean("Run Camera Stuff", true)) {
          myCameraMaster.update();
          Thread.yield();
        } else {
          Thread.yield();
          
          try {
            Thread.sleep(2000);
          } catch(InterruptedException e) {
            e.printStackTrace();
          }
        }
      }
    });
    cameraThread.start();*/

    SmartDashboard.putNumber("Auto (1 = Left of Speaker, 2 = Center, 3 = Right)", 0);
    SmartDashboard.putString("Auto String: ", "0");
  }

  //Every 20ms
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Arm Angle: ", myArmMaster.getAngle());
    SmartDashboard.putNumber("LU Angle: ", mySwerveMaster.leftUpModule.getAbsoluteTurnPosition());
    SmartDashboard.putNumber("LD Angle: ", mySwerveMaster.leftDownModule.getAbsoluteTurnPosition());
    SmartDashboard.putNumber("RU Angle: ", mySwerveMaster.rightUpModule.getAbsoluteTurnPosition());
    SmartDashboard.putNumber("RD Angle: ", mySwerveMaster.rightDownModule.getAbsoluteTurnPosition());
    SmartDashboard.putNumber("Current Battery Voltage: ", Math.round(RobotController.getBatteryVoltage() * 10) / 10d);
  }

  @Override
  public void autonomousInit() {
    mySwerveMaster.blue = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
    myArmMaster.blue = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;

    if(!mySwerveMaster.blue) {
      for(int i = 3; i < 8; i++) {
        Constants.autoConstants.notePositions[i][2] += 180;
      }
    }

    if(SmartDashboard.getString("Auto String: ", "0").equals("0")) {
      while (!(SmartDashboard.getString("Launcher Status: ", "NOT YET").equals("LAUNCHER READY") &&  
        SmartDashboard.getString("Arm Angle Status: ", "NOT YET").equals("ARM ANGLE READY"))) {
        myArmMaster.updateVoltage();
        mySwerveMaster.updateRobotPosition(mySwerveMaster.getReducedAngle());
        mySwerveMaster.stop();
        myArmMaster.stopIntake();

        //Prep arm for scoring into speaker
        //LauncherInfo info = myArmMaster.myLauncherInfoMaster.get(Math.sqrt(Math.pow(mySwerveMaster.getRobotPosition()[0] - Constants.gameConstants.speakerX, 2) + Math.pow(mySwerveMaster.getRobotPosition()[1] - (mySwerveMaster.blue ? Constants.gameConstants.blueConstants.speakerY : Constants.gameConstants.redConstants.speakerY), 2)), 0, myArmMaster.maxVoltage);
        myArmMaster.prepLauncher(0.31);

        if(Math.abs(69d - myArmMaster.getAngle()) < Constants.armConstants.angleTolerance) {
          SmartDashboard.putString("Arm Angle Status: ", "ARM ANGLE READY");
          myArmMaster.changeAngle(0);
        } else {
          SmartDashboard.putString("Arm Angle Status: ", "NOT YET");
          myArmMaster.changeAngle((69d - myArmMaster.getAngle()) / 22.5);
        }
      }

      double time = Timer.getFPGATimestamp();

      while(Timer.getFPGATimestamp() - time < 0.1) {
        myArmMaster.prepLauncher(0.31);
        myArmMaster.fireLauncher();
      }

      double backupTime = Constants.autoConstants.backupTimeLeft;
      double turnTime = Constants.autoConstants.turnTimeLeft;
      int autoNum = (int) SmartDashboard.getNumber("Auto (1 = Left of Speaker, 2 = Center, 3 = Right)", 1);

      if(autoNum == 0) {
        backupTime = 0d;
        turnTime = 0d;
      } else if(autoNum == 2) {
        turnTime = Constants.autoConstants.turnTimeCenter;
        backupTime = Constants.autoConstants.backupTimeCenter;
      } else if(autoNum == 3) {
        turnTime = Constants.autoConstants.turnTimeRight;
        backupTime = Constants.autoConstants.backupTimeRight;
      }

      time = Timer.getFPGATimestamp();

      while(Timer.getFPGATimestamp() - time < turnTime) {
        myArmMaster.stop();
        mySwerveMaster.drive(new double[]{0, 0, (autoNum == 3 ? -1 : 1) * Constants.autoConstants.turnSpeed}, new double[]{mySwerveMaster.leftUpModule.getAbsoluteTurnPosition(), 
          mySwerveMaster.leftDownModule.getAbsoluteTurnPosition(), mySwerveMaster.rightUpModule.getAbsoluteTurnPosition(), 
          mySwerveMaster.rightDownModule.getAbsoluteTurnPosition()}, mySwerveMaster.getReducedAngle(), 1.0, 1.0);
      }

      time = Timer.getFPGATimestamp();

      while(Timer.getFPGATimestamp() - time < backupTime) {
        myArmMaster.stop();
        mySwerveMaster.drive(new double[]{0, -0.25, 0}, new double[]{mySwerveMaster.leftUpModule.getAbsoluteTurnPosition(), 
          mySwerveMaster.leftDownModule.getAbsoluteTurnPosition(), mySwerveMaster.rightUpModule.getAbsoluteTurnPosition(), 
          mySwerveMaster.rightDownModule.getAbsoluteTurnPosition()}, mySwerveMaster.getReducedAngle(), 1.0, 1.0);
      }

      time = Timer.getFPGATimestamp();

      while(Timer.getFPGATimestamp() - time < 1) {
        mySwerveMaster.stop();
        myArmMaster.stop();
      }

      return;
    }

    /*String[] numberStrings = SmartDashboard.getString("Auto String: ", "0").split(",");
    int[] numbers = new int[numberStrings.length];

    for(int i = 0; i < numberStrings.length; i++) {
      numbers[i] = Integer.valueOf(numberStrings[i]);
    }

    int homeNote = numbers[0];

    if(homeNote == 1) {
      mySwerveMaster.resetPose(Constants.autoConstants.blueRight[0], Constants.autoConstants.blueRight[1], Constants.autoConstants.blueRight[2]);
    } else if(homeNote == 2) {
      mySwerveMaster.resetPose(Constants.autoConstants.blueCenter[0], Constants.autoConstants.blueCenter[1], Constants.autoConstants.blueCenter[2]);
    } else if(homeNote == 3) {
      mySwerveMaster.resetPose(Constants.autoConstants.blueLeft[0], Constants.autoConstants.blueLeft[1], Constants.autoConstants.blueLeft[2]);
    } else if(homeNote == 4) {
      mySwerveMaster.resetPose(Constants.autoConstants.redLeft[0], Constants.autoConstants.redLeft[1], Constants.autoConstants.redLeft[2]);
    } else if(homeNote == 5) {
      mySwerveMaster.resetPose(Constants.autoConstants.redCenter[0], Constants.autoConstants.redCenter[1], Constants.autoConstants.redCenter[2]);
    } else if(homeNote == 6) {
      mySwerveMaster.resetPose(Constants.autoConstants.redRight[0], Constants.autoConstants.redRight[1], Constants.autoConstants.redRight[2]);
    }

    shootNote();

    autoMoveIntake(Constants.autoConstants.notePositions[homeNote - 1]);
    autoMoveFire(Constants.autoConstants.notePositions[homeNote - 1]);
    shootNote();

    for(int i = 1; i < numbers.length; i++) {
      int targetNote = numbers[i];

      if(homeNote == 1) {
        switch(targetNote) {
          case 2:
            autoMoveIntake(Constants.autoConstants.notePositions[1]);
            autoMovePrep(Constants.autoConstants.notePositions[1]);
            shootNote();
            homeNote = 2;
            break;
          case 3:
            autoMoveIntake(Constants.autoConstants.notePositions[2]);
            autoMovePrep(Constants.autoConstants.notePositions[2]);
            shootNote();
            homeNote = 3;
            break;
          case 4:
            autoMoveIntake(Constants.autoConstants.notePositions[3]);
            autoMovePrep(Constants.autoConstants.notePositions[0]);
            shootNote();
            break;
          case 5:
            autoMoveIntake(Constants.autoConstants.notePositions[4]);
            autoMovePrep(Constants.autoConstants.notePositions[0]);
            shootNote();
            break;
          case 6:
            autoMove(new double[]{Constants.autoConstants.notePositions[0][0], Constants.gameConstants.blueConstants.trapY1, Constants.autoConstants.notePositions[5][2]});
            autoMoveIntake(Constants.autoConstants.notePositions[5]);
            autoMovePrep(new double[]{Constants.autoConstants.notePositions[0][0], Constants.gameConstants.blueConstants.trapY1});
            autoMovePrep(Constants.autoConstants.notePositions[0]);
            shootNote();
            break;
          case 7:
            autoMove(new double[]{Constants.autoConstants.notePositions[3][0], Constants.gameConstants.blueConstants.trapY1, Constants.autoConstants.notePositions[6][2]});
            autoMoveIntake(Constants.autoConstants.notePositions[6]);
            autoMovePrep(new double[]{Constants.autoConstants.notePositions[3][0], Constants.gameConstants.blueConstants.trapY1});
            autoMovePrep(Constants.autoConstants.notePositions[0]);
            shootNote();
            break;
          case 8:
            autoMove(new double[]{Constants.autoConstants.notePositions[3][0], Constants.gameConstants.blueConstants.trapY1, Constants.autoConstants.notePositions[7][2]});
            autoMoveIntake(Constants.autoConstants.notePositions[7]);
            autoMovePrep(new double[]{Constants.autoConstants.notePositions[3][0], Constants.gameConstants.blueConstants.trapY1});
            autoMovePrep(Constants.autoConstants.notePositions[0]);
            shootNote();
            break;
        }
      } else if(homeNote == 2) {
          switch(targetNote) {
            case 1:
              autoMoveIntake(Constants.autoConstants.notePositions[0]);
              autoMovePrep(Constants.autoConstants.notePositions[0]);
              shootNote();
              homeNote = 1;
              break;
            case 3:
              autoMoveIntake(Constants.autoConstants.notePositions[2]);
              autoMovePrep(Constants.autoConstants.notePositions[2]);
              shootNote();
              homeNote = 3;
              break;
            case 4:
              autoMoveIntake(Constants.autoConstants.notePositions[3]);
              autoMovePrep(Constants.autoConstants.notePositions[1]);
              shootNote();
              break;
            case 5:
              autoMove(new double[]{Constants.autoConstants.notePositions[0][0], Constants.gameConstants.blueConstants.trapY1, Constants.autoConstants.notePositions[4][2]});
              autoMoveIntake(Constants.autoConstants.notePositions[4]);
              autoMovePrep(new double[]{Constants.autoConstants.notePositions[0][0], Constants.gameConstants.blueConstants.trapY1});
              autoMovePrep(Constants.autoConstants.notePositions[1]);
              shootNote();
              break;
            case 6:
              autoMove(new double[]{Constants.autoConstants.notePositions[0][0], Constants.gameConstants.blueConstants.trapY1, Constants.autoConstants.notePositions[5][2]});
              autoMoveIntake(Constants.autoConstants.notePositions[5]);
              autoMovePrep(new double[]{Constants.autoConstants.notePositions[0][0], Constants.gameConstants.blueConstants.trapY1});
              autoMovePrep(Constants.autoConstants.notePositions[1]);
              shootNote();
              break;
            case 7:
              autoMove(new double[]{Constants.autoConstants.notePositions[3][0], Constants.gameConstants.blueConstants.trapY1, Constants.autoConstants.notePositions[6][2]});
              autoMoveIntake(Constants.autoConstants.notePositions[6]);
              autoMovePrep(new double[]{Constants.autoConstants.notePositions[3][0], Constants.gameConstants.blueConstants.trapY1});
              autoMovePrep(Constants.autoConstants.notePositions[1]);
              shootNote();
              break;
            case 8:
              autoMove(new double[]{Constants.autoConstants.notePositions[3][0], Constants.gameConstants.blueConstants.trapY1, Constants.autoConstants.notePositions[7][2]});
              autoMoveIntake(Constants.autoConstants.notePositions[7]);
              autoMovePrep(new double[]{Constants.autoConstants.notePositions[3][0], Constants.gameConstants.blueConstants.trapY1});
              autoMovePrep(Constants.autoConstants.notePositions[1]);
              shootNote();
              break;
          }
        } else if(homeNote == 3) {
        switch(targetNote) {
          case 1:
            autoMoveIntake(Constants.autoConstants.notePositions[0]);
            autoMovePrep(Constants.autoConstants.notePositions[0]);
            shootNote();
            homeNote = 1;
            break;
          case 2:
            autoMoveIntake(Constants.autoConstants.notePositions[1]);
            autoMovePrep(Constants.autoConstants.notePositions[1]);
            shootNote();
            homeNote = 2;
            break;
          case 4:
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.notePositions[3][1], Constants.autoConstants.notePositions[7][2]});
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.blueConstants.trap1Y, Constants.autoConstants.notePositions[7][2]});
            autoMoveIntake(Constants.autoConstants.notePositions[3]);
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.blueConstants.trap1Y, Constants.autoConstants.notePositions[7][2]});
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.notePositions[3][1], Constants.autoConstants.notePositions[7][2]});
            autoMovePrep(Constants.autoConstants.notePositions[2]);
            shootNote();
            break;
          case 5:
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.notePositions[3][1], Constants.autoConstants.notePositions[7][2]});
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.blueConstants.trap1Y, Constants.autoConstants.notePositions[7][2]});
            autoMoveIntake(Constants.autoConstants.notePositions[4]);
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.blueConstants.trap1Y, Constants.autoConstants.notePositions[7][2]});
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.notePositions[3][1], Constants.autoConstants.notePositions[7][2]});
            autoMovePrep(Constants.autoConstants.notePositions[2]);
            shootNote();
            break;
          case 6:
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.notePositions[3][1], Constants.autoConstants.notePositions[7][2]});
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.blueConstants.trap1Y, Constants.autoConstants.notePositions[7][2]});
            autoMoveIntake(Constants.autoConstants.notePositions[5]);
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.blueConstants.trap1Y, Constants.autoConstants.notePositions[7][2]});
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.notePositions[3][1], Constants.autoConstants.notePositions[7][2]});
            autoMovePrep(Constants.autoConstants.notePositions[2]);
            shootNote();
            break;
          case 7:
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.notePositions[3][1], Constants.autoConstants.notePositions[6][2]});
            autoMoveIntake(Constants.autoConstants.notePositions[6]);
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.notePositions[3][1], Constants.autoConstants.notePositions[6][2]});
            autoMovePrep(Constants.autoConstants.notePositions[2]);
            shootNote();
            break;
          case 8:
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.notePositions[3][1], Constants.autoConstants.notePositions[7][2]});
            autoMoveIntake(Constants.autoConstants.notePositions[7]);
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.notePositions[3][1], Constants.autoConstants.notePositions[7][2]});
            autoMovePrep(Constants.autoConstants.notePositions[2]);
            shootNote();
            break;
        }
      } else if(homeNote == 9) {
        switch(targetNote) {
          case 10:
            autoMoveIntake(Constants.autoConstants.notePositions[9]);
            autoMovePrep(Constants.autoConstants.notePositions[9]);
            shootNote();
            homeNote = 10;
            break;
          case 11:
            autoMoveIntake(Constants.autoConstants.notePositions[10]);
            autoMovePrep(Constants.autoConstants.notePositions[10]);
            shootNote();
            homeNote = 11;
            break;
          case 4:
            autoMoveIntake(Constants.autoConstants.notePositions[3]);
            autoMovePrep(Constants.autoConstants.notePositions[8]);
            shootNote();
            break;
          case 5:
            autoMoveIntake(Constants.autoConstants.notePositions[4]);
            autoMovePrep(Constants.autoConstants.notePositions[8]);
            shootNote();
            break;
          case 6:
            autoMove(new double[]{Constants.autoConstants.notePositions[8][0], Constants.gameConstants.redConstants.trapY1, Constants.autoConstants.notePositions[5][2]});
            autoMoveIntake(Constants.autoConstants.notePositions[5]);
            autoMovePrep(new double[]{Constants.autoConstants.notePositions[8][0], Constants.gameConstants.redConstants.trapY1});
            autoMovePrep(Constants.autoConstants.notePositions[8]);
            shootNote();
            break;
          case 7:
            autoMove(new double[]{Constants.autoConstants.notePositions[3][0], Constants.gameConstants.redConstants.trapY1, Constants.autoConstants.notePositions[6][2]});
            autoMoveIntake(Constants.autoConstants.notePositions[6]);
            autoMovePrep(new double[]{Constants.autoConstants.notePositions[3][0], Constants.gameConstants.redConstants.trapY1});
            autoMovePrep(Constants.autoConstants.notePositions[8]);
            shootNote();
            break;
          case 8:
            autoMove(new double[]{Constants.autoConstants.notePositions[3][0], Constants.gameConstants.redConstants.trapY1, Constants.autoConstants.notePositions[7][2]});
            autoMoveIntake(Constants.autoConstants.notePositions[7]);
            autoMovePrep(new double[]{Constants.autoConstants.notePositions[3][0], Constants.gameConstants.redConstants.trapY1});
            autoMovePrep(Constants.autoConstants.notePositions[8]);
            shootNote();
            break;
        }
      } else if(homeNote == 10) {
          switch(targetNote) {
            case 9:
              autoMoveIntake(Constants.autoConstants.notePositions[8]);
              autoMovePrep(Constants.autoConstants.notePositions[8]);
              shootNote();
              homeNote = 9;
              break;
            case 11:
              autoMoveIntake(Constants.autoConstants.notePositions[10]);
              autoMovePrep(Constants.autoConstants.notePositions[10]);
              shootNote();
              homeNote = 11;
              break;
            case 4:
              autoMoveIntake(Constants.autoConstants.notePositions[3]);
              autoMovePrep(Constants.autoConstants.notePositions[9]);
              shootNote();
              break;
            case 5:
              autoMove(new double[]{Constants.autoConstants.notePositions[8][0], Constants.gameConstants.redConstants.trapY1, Constants.autoConstants.notePositions[4][2]});
              autoMoveIntake(Constants.autoConstants.notePositions[4]);
              autoMovePrep(new double[]{Constants.autoConstants.notePositions[8][0], Constants.gameConstants.redConstants.trapY1});
              autoMovePrep(Constants.autoConstants.notePositions[9]);
              shootNote();
              break;
            case 6:
              autoMove(new double[]{Constants.autoConstants.notePositions[8][0], Constants.gameConstants.redConstants.trapY1, Constants.autoConstants.notePositions[5][2]});
              autoMoveIntake(Constants.autoConstants.notePositions[5]);
              autoMovePrep(new double[]{Constants.autoConstants.notePositions[8][0], Constants.gameConstants.redConstants.trapY1});
              autoMovePrep(Constants.autoConstants.notePositions[9]);
              shootNote();
              break;
            case 7:
              autoMove(new double[]{Constants.autoConstants.notePositions[3][0], Constants.gameConstants.redConstants.trapY1, Constants.autoConstants.notePositions[6][2]});
              autoMoveIntake(Constants.autoConstants.notePositions[6]);
              autoMovePrep(new double[]{Constants.autoConstants.notePositions[3][0], Constants.gameConstants.redConstants.trapY1});
              autoMovePrep(Constants.autoConstants.notePositions[9]);
              shootNote();
              break;
            case 8:
              autoMove(new double[]{Constants.autoConstants.notePositions[3][0], Constants.gameConstants.redConstants.trapY1, Constants.autoConstants.notePositions[7][2]});
              autoMoveIntake(Constants.autoConstants.notePositions[7]);
              autoMovePrep(new double[]{Constants.autoConstants.notePositions[3][0], Constants.gameConstants.redConstants.trapY1});
              autoMovePrep(Constants.autoConstants.notePositions[9]);
              shootNote();
              break;
          }
        } else if(homeNote == 11) {
        switch(targetNote) {
          case 9:
            autoMoveIntake(Constants.autoConstants.notePositions[8]);
            autoMovePrep(Constants.autoConstants.notePositions[8]);
            shootNote();
            homeNote = 9;
            break;
          case 10:
            autoMoveIntake(Constants.autoConstants.notePositions[9]);
            autoMovePrep(Constants.autoConstants.notePositions[9]);
            shootNote();
            homeNote = 10;
            break;
          case 4:
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.notePositions[10][1], Constants.autoConstants.notePositions[7][2]});
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.redConstants.trap1Y, Constants.autoConstants.notePositions[7][2]});
            autoMoveIntake(Constants.autoConstants.notePositions[3]);
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.redConstants.trap1Y, Constants.autoConstants.notePositions[7][2]});
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.notePositions[10][1], Constants.autoConstants.notePositions[7][2]});
            autoMovePrep(Constants.autoConstants.notePositions[10]);
            shootNote();
            break;
          case 5:
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.notePositions[10][1], Constants.autoConstants.notePositions[7][2]});
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.redConstants.trap1Y, Constants.autoConstants.notePositions[7][2]});
            autoMoveIntake(Constants.autoConstants.notePositions[4]);
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.redConstants.trap1Y, Constants.autoConstants.notePositions[7][2]});
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.notePositions[10][1], Constants.autoConstants.notePositions[7][2]});
            autoMovePrep(Constants.autoConstants.notePositions[10]);
            shootNote();
            break;
          case 6:
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.notePositions[10][1], Constants.autoConstants.notePositions[7][2]});
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.redConstants.trap1Y, Constants.autoConstants.notePositions[7][2]});
            autoMoveIntake(Constants.autoConstants.notePositions[5]);
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.redConstants.trap1Y, Constants.autoConstants.notePositions[7][2]});
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.notePositions[10][1], Constants.autoConstants.notePositions[7][2]});
            autoMovePrep(Constants.autoConstants.notePositions[10]);
            shootNote();
            break;
          case 7:
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.notePositions[10][1], Constants.autoConstants.notePositions[6][2]});
            autoMoveIntake(Constants.autoConstants.notePositions[6]);
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.notePositions[10][1], Constants.autoConstants.notePositions[6][2]});
            autoMovePrep(Constants.autoConstants.notePositions[10]);
            shootNote();
            break;
          case 8:
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.notePositions[10][1], Constants.autoConstants.notePositions[7][2]});
            autoMoveIntake(Constants.autoConstants.notePositions[7]);
            autoMove(new double[]{Constants.autoConstants.notePositions[7][0], Constants.autoConstants.notePositions[10][1], Constants.autoConstants.notePositions[7][2]});
            autoMovePrep(Constants.autoConstants.notePositions[10]);
            shootNote();
            break;
        }
      }
    }

    //SmartDashboard.putString("Current mode: ", "autonomousInit");
    /*autoNum = (int) SmartDashboard.getNumber("Auto (1 = Left of Speaker, 2 = Center, 3 = Right)", 0);
    mySwerveMaster.blue = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
    myArmMaster.blue = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;

    if(!mySwerveMaster.blue) {
      for(int i = 3; i < 8; i++) {
        Constants.autoConstants.notePositions[i][2] += 180;
      }
    }

    if(autoNum == 0) {
        while (!(SmartDashboard.getString("Launcher Status: ", "NOT YET").equals("LAUNCHER READY") &&  
        SmartDashboard.getString("Arm Angle Status: ", "NOT YET").equals("ARM ANGLE READY"))) {
        myArmMaster.updateVoltage();
        mySwerveMaster.updateRobotPosition(mySwerveMaster.getReducedAngle());
        mySwerveMaster.stop();
        myArmMaster.stopIntake();

        //Prep arm for scoring into speaker
        //LauncherInfo info = myArmMaster.myLauncherInfoMaster.get(Math.sqrt(Math.pow(mySwerveMaster.getRobotPosition()[0] - Constants.gameConstants.speakerX, 2) + Math.pow(mySwerveMaster.getRobotPosition()[1] - (mySwerveMaster.blue ? Constants.gameConstants.blueConstants.speakerY : Constants.gameConstants.redConstants.speakerY), 2)), 0, myArmMaster.maxVoltage);
        myArmMaster.prepLauncher(0.31);

        if(Math.abs(69d - myArmMaster.getAngle()) < Constants.armConstants.angleTolerance) {
          SmartDashboard.putString("Arm Angle Status: ", "ARM ANGLE READY");
          myArmMaster.changeAngle(0);
        } else {
          SmartDashboard.putString("Arm Angle Status: ", "NOT YET");
          myArmMaster.changeAngle((69d - myArmMaster.getAngle()) / 22.5);
        }
      }

      myArmMaster.fireLauncher();
    } else if(autoNum == 1) {
      if(mySwerveMaster.blue) {
        mySwerveMaster.resetPose(Constants.autoConstants.blueLeft[0], Constants.autoConstants.blueLeft[1], Constants.autoConstants.blueLeft[2]);
      } else {
        mySwerveMaster.resetPose(Constants.autoConstants.redLeft[0], Constants.autoConstants.redLeft[1], Constants.autoConstants.redLeft[2]);
      }

      shootNote();

      if(mySwerveMaster.blue) {
        autoMoveIntake(Constants.autoConstants.notePositions[2]);
        autoMoveFire(Constants.autoConstants.notePositions[2]);
        autoMove(new double[]{Constants.autoConstants.notePositions[7][0], mySwerveMaster.getRobotPosition()[1], Constants.autoConstants.notePositions[7][2]});
        autoMoveIntake(Constants.autoConstants.notePositions[7]);
        autoMovePrep(new double[]{Constants.autoConstants.notePositions[7][0], mySwerveMaster.getRobotPosition()[1]});
        autoMovePrep(Constants.autoConstants.notePositions[2]);
      } else {
        autoMoveIntake(Constants.autoConstants.notePositions[8]);
        autoMoveFire(Constants.autoConstants.notePositions[8]);
        autoMoveIntake(Constants.autoConstants.notePositions[3]);
        autoMovePrep(Constants.autoConstants.notePositions[8]);
      }

      shootNote();
    } else if(autoNum == 2) {
      if(mySwerveMaster.blue) {
        mySwerveMaster.resetPose(Constants.autoConstants.blueCenter[0], Constants.autoConstants.blueCenter[1], Constants.autoConstants.blueCenter[2]);
      } else {
        mySwerveMaster.resetPose(Constants.autoConstants.redCenter[0], Constants.autoConstants.redCenter[1], Constants.autoConstants.redCenter[2]);
      }

      shootNote();

      if(mySwerveMaster.blue) {
        autoMoveIntake(Constants.autoConstants.notePositions[1]);
        autoMoveFire(Constants.autoConstants.notePositions[1]);
        autoMove(new double[]{Constants.autoConstants.notePositions[4][0], Constants.gameConstants.blueConstants.trapY1, Constants.autoConstants.notePositions[4][2]});
        autoMoveIntake(Constants.autoConstants.notePositions[4]);
        autoMovePrep(new double[]{Constants.autoConstants.notePositions[4][0], Constants.gameConstants.blueConstants.trapY1});
        autoMovePrep(Constants.autoConstants.notePositions[1]);
      } else {
        autoMoveIntake(Constants.autoConstants.notePositions[9]);
        autoMoveFire(Constants.autoConstants.notePositions[9]);
        autoMove(new double[]{Constants.autoConstants.notePositions[4][0], Constants.gameConstants.redConstants.trapY1, Constants.autoConstants.notePositions[4][2]});
        autoMoveIntake(Constants.autoConstants.notePositions[4]);
        autoMovePrep(new double[]{Constants.autoConstants.notePositions[4][0], Constants.gameConstants.redConstants.trapY1});
        autoMovePrep(Constants.autoConstants.notePositions[9]);
      }

      shootNote();
    } else if(autoNum == 3) {
      if(mySwerveMaster.blue) {
        mySwerveMaster.resetPose(Constants.autoConstants.blueRight[0], Constants.autoConstants.blueRight[1], Constants.autoConstants.blueRight[2]);
      } else {
        mySwerveMaster.resetPose(Constants.autoConstants.redRight[0], Constants.autoConstants.redRight[1], Constants.autoConstants.redRight[2]);
      }

      shootNote();

      if(mySwerveMaster.blue) {
        autoMoveIntake(Constants.autoConstants.notePositions[0]);
        autoMoveFire(Constants.autoConstants.notePositions[0]);
        autoMoveIntake(Constants.autoConstants.notePositions[3]);
        autoMovePrep(Constants.autoConstants.notePositions[0]);
      } else {
        autoMoveIntake(Constants.autoConstants.notePositions[10]);
        autoMoveFire(Constants.autoConstants.notePositions[10]);
        autoMove(new double[]{Constants.autoConstants.notePositions[7][0], mySwerveMaster.getRobotPosition()[1], Constants.autoConstants.notePositions[7][2]});
        autoMoveIntake(Constants.autoConstants.notePositions[7]);
        autoMovePrep(new double[]{Constants.autoConstants.notePositions[7][0], mySwerveMaster.getRobotPosition()[1]});
        autoMovePrep(Constants.autoConstants.notePositions[10]);
      }

      shootNote();
    }*/

    /*mySwerveMaster.resetPose(Constants.autoConstants.redLeft[0], Constants.autoConstants.redLeft[1], Constants.autoConstants.redLeft[2]);
    shootNote();*/

    /*if (autoNum == 1) {
      if (mySwerveMaster.blue) {
        mySwerveMaster.resetPose(Constants.autoConstants.blueLeft[0], Constants.autoConstants.blueLeft[1], Constants.autoConstants.blueLeft[2]);
      } else {
        mySwerveMaster.resetPose(Constants.autoConstants.redLeft[0], Constants.autoConstants.redLeft[1], Constants.autoConstants.redLeft[2]);
      }

      shootNote();

      //Turn and move to intake note for shooting into speaker
      double desiredAngle = mySwerveMaster.blue ? 0 : 180;
      double desiredTurnInput = mySwerveMaster.turnPIDController.calculate(mySwerveMaster.getReducedAngle(), desiredAngle) / 180d;
      double time = Timer.getFPGATimestamp();
      while (Timer.getFPGATimestamp() - time < Constants.autoConstants.backupTimeLeft) {
        myArmMaster.updateVoltage();
        mySwerveMaster.updateRobotPosition(mySwerveMaster.getReducedAngle());
        mySwerveMaster.drive(new double[]{0, Constants.autoConstants.backupSpeedLeft, desiredTurnInput}, 
        new double[]{mySwerveMaster.leftUpModule.getAbsoluteTurnPosition(), mySwerveMaster.leftDownModule.getAbsoluteTurnPosition(), mySwerveMaster.rightUpModule.getAbsoluteTurnPosition(), mySwerveMaster.rightDownModule.getAbsoluteTurnPosition()},
        mySwerveMaster.getReducedAngle(), Constants.autoConstants.backupSpeedLeft, Constants.autoConstants.turnSpeed);
        myArmMaster.intakeBottom();
        desiredTurnInput = mySwerveMaster.turnPIDController.calculate(mySwerveMaster.getReducedAngle(), desiredAngle) / 180d;
      }

      mySwerveMaster.stop();

      double relativeX = Constants.gameConstants.speakerX - mySwerveMaster.getRobotPosition()[0];
      double relativeY = (mySwerveMaster.blue ? Constants.gameConstants.blueConstants.speakerY : Constants.gameConstants.redConstants.speakerY) - mySwerveMaster.getRobotPosition()[1];
      double originalAngle = ((Math.atan2(relativeY, relativeX) - 3 * Math.PI / 2)) * 180 / Math.PI;
      desiredAngle = mySwerveMaster.blue ? Math.max(Math.min(Constants.gameConstants.blueConstants.maxSpeakerAngle, originalAngle), Constants.gameConstants.blueConstants.minSpeakerAngle) : Math.max(Math.min(Constants.gameConstants.redConstants.maxSpeakerAngle, originalAngle), Constants.gameConstants.redConstants.minSpeakerAngle);
      desiredTurnInput = mySwerveMaster.turnPIDController.calculate(mySwerveMaster.getReducedAngle(), desiredAngle) / 180d;
      while (Math.abs(desiredTurnInput - mySwerveMaster.getReducedAngle()) >= Constants.autoConstants.turnTolerance) {
        myArmMaster.updateVoltage();
        myArmMaster.stopIntake();
        mySwerveMaster.updateRobotPosition(mySwerveMaster.getReducedAngle());
        mySwerveMaster.drive(new double[]{0, 0, desiredTurnInput}, 
        new double[]{mySwerveMaster.leftUpModule.getAbsoluteTurnPosition(), mySwerveMaster.leftDownModule.getAbsoluteTurnPosition(), mySwerveMaster.rightUpModule.getAbsoluteTurnPosition(), mySwerveMaster.rightDownModule.getAbsoluteTurnPosition()},
         mySwerveMaster.getReducedAngle(), 0, Constants.autoConstants.turnSpeed);
        desiredTurnInput = mySwerveMaster.turnPIDController.calculate(mySwerveMaster.getReducedAngle(), desiredAngle) / 180d;
      }
    } else if (autoNum == 2) { //Center start for auto.
      if (mySwerveMaster.blue) {
        mySwerveMaster.resetPose(Constants.autoConstants.blueCenter[0], Constants.autoConstants.blueCenter[1], Constants.autoConstants.blueCenter[2]);
      } else {
        mySwerveMaster.resetPose(Constants.autoConstants.redCenter[0], Constants.autoConstants.redCenter[1], Constants.autoConstants.redCenter[2]);
      }

      shootNote();
      
      double time = Timer.getFPGATimestamp();
      while (Timer.getFPGATimestamp() - time < Constants.autoConstants.backupTimeCenter) {
        myArmMaster.updateVoltage();
        mySwerveMaster.updateRobotPosition(mySwerveMaster.getReducedAngle());
        mySwerveMaster.drive(new double[]{0, Constants.autoConstants.backupSpeedCenter, 0}, 
        new double[]{mySwerveMaster.leftUpModule.getAbsoluteTurnPosition(), mySwerveMaster.leftDownModule.getAbsoluteTurnPosition(), mySwerveMaster.rightUpModule.getAbsoluteTurnPosition(), mySwerveMaster.rightDownModule.getAbsoluteTurnPosition()},
         mySwerveMaster.getReducedAngle(), Constants.autoConstants.backupSpeedCenter, 0);
         myArmMaster.intakeBottom();
      }
    } else if (autoNum == 3) {
      if (mySwerveMaster.blue) {
        mySwerveMaster.resetPose(Constants.autoConstants.blueRight[0], Constants.autoConstants.blueRight[1], Constants.autoConstants.blueRight[2]);
      } else {
        mySwerveMaster.resetPose(Constants.autoConstants.redRight[0], Constants.autoConstants.redRight[1], Constants.autoConstants.redRight[2]);
      }

      shootNote();
      
      //Turn and move to intake note for shooting into speaker
      double desiredAngle = mySwerveMaster.blue ? 0 : 180;
      double desiredTurnInput = mySwerveMaster.turnPIDController.calculate(mySwerveMaster.getReducedAngle(), desiredAngle) / 180d;
      double time = Timer.getFPGATimestamp();
      while (Timer.getFPGATimestamp() - time < Constants.autoConstants.backupTimeRight) {
        mySwerveMaster.updateRobotPosition(mySwerveMaster.getReducedAngle());
        mySwerveMaster.drive(new double[]{0, Constants.autoConstants.backupSpeedRight, desiredTurnInput}, 
        new double[]{mySwerveMaster.leftUpModule.getAbsoluteTurnPosition(), mySwerveMaster.leftDownModule.getAbsoluteTurnPosition(), mySwerveMaster.rightUpModule.getAbsoluteTurnPosition(), mySwerveMaster.rightDownModule.getAbsoluteTurnPosition()},
         mySwerveMaster.getReducedAngle(), Constants.autoConstants.backupSpeedRight, Constants.autoConstants.turnSpeed);
         myArmMaster.intakeBottom();
        desiredTurnInput = mySwerveMaster.turnPIDController.calculate(mySwerveMaster.getReducedAngle(), desiredAngle) / 180d;
      }

      mySwerveMaster.stop();

      double relativeX = Constants.gameConstants.speakerX - mySwerveMaster.getRobotPosition()[0];
      double relativeY = (mySwerveMaster.blue ? Constants.gameConstants.blueConstants.speakerY : Constants.gameConstants.redConstants.speakerY) - mySwerveMaster.getRobotPosition()[1];
      double originalAngle = ((Math.atan2(relativeY, relativeX) - 3 * Math.PI / 2)) * 180 / Math.PI;
      desiredAngle = mySwerveMaster.blue ? Math.max(Math.min(Constants.gameConstants.blueConstants.maxSpeakerAngle, originalAngle), Constants.gameConstants.blueConstants.minSpeakerAngle) : Math.max(Math.min(Constants.gameConstants.redConstants.maxSpeakerAngle, originalAngle), Constants.gameConstants.redConstants.minSpeakerAngle);
      desiredTurnInput = mySwerveMaster.turnPIDController.calculate(mySwerveMaster.getReducedAngle(), desiredAngle) / 180d;
      while (Math.abs(desiredTurnInput - mySwerveMaster.getReducedAngle()) >= Constants.autoConstants.turnTolerance) {
        myArmMaster.stopIntake();
        mySwerveMaster.updateRobotPosition(mySwerveMaster.getReducedAngle());
        mySwerveMaster.drive(new double[]{0, 0, desiredTurnInput}, 
        new double[]{mySwerveMaster.leftUpModule.getAbsoluteTurnPosition(), mySwerveMaster.leftDownModule.getAbsoluteTurnPosition(), mySwerveMaster.rightUpModule.getAbsoluteTurnPosition(), mySwerveMaster.rightDownModule.getAbsoluteTurnPosition()},
         mySwerveMaster.getReducedAngle(), 0, Constants.autoConstants.turnSpeed);
        desiredTurnInput = mySwerveMaster.turnPIDController.calculate(mySwerveMaster.getReducedAngle(), desiredAngle) / 180d;
      }
    }

    if(autoNum != 0) {
      mySwerveMaster.stop();
      //shootNote();
    }*/
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    if(driveController.getTouchpadPressed()) {
      driveControllerFactor = 0d;
    } else if(driveController.getL2Button()) {
      driveControllerFactor = Math.min(1, driveControllerFactor + 0.01);
    } else if(driveController.getL1Button()) {
      driveControllerFactor = Math.max(0.1, driveControllerFactor - 0.01);
    }

    if(driveController.getPSButtonPressed()) {
      turnFactor = 0d;
    } else if(driveController.getR2Button()) {
      turnFactor = Math.min(1, turnFactor + 0.01);
    } else if(driveController.getR1Button()) {
      turnFactor = Math.max(0.1, turnFactor - 0.01);
    }

    if(armController.getTouchpadPressed()) {
      angleFactor = 0d;
    } else if(armController.getL2Button()) {
      angleFactor = Math.min(1, angleFactor + 0.01);
    } else if(armController.getL1Button()) {
      angleFactor = Math.max(0.1, angleFactor - 0.01);
    }

    if(armController.getPSButtonPressed()) {
      fireFactor = 0d;
    } else if(armController.getR2Button()) {
      fireFactor = Math.min(1, fireFactor + 0.0025);
    } else if(armController.getR1Button()) {
      fireFactor = Math.max(0.1, fireFactor - 0.0025);
    }

    //Accelerometer will need to be reset due to inaccuracies accumulating
    //Odometry - Resets the origin and angle to the current position and angle of the robot
    if (driveController.getShareButtonPressed()) {
      mySwerveMaster.resetPose(0, 0, 360);
    }

    mySwerveMaster.update(driveController, driveControllerFactor, turnFactor);

    //myHookMaster.update(driveController.getPOV());
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

    if(SmartDashboard.getString("Launcher Status: ", "NOT YET").equals("LAUNCHER READY") && 
      SmartDashboard.getString("Heading Status: ", "NOT YET").equals("HEADING READY") && 
      SmartDashboard.getString("Arm Angle Status: ", "NOT YET").equals("ARM ANGLE READY") && 
      SmartDashboard.getString("Distance Range Status: ", "NOT YET").equals("DISTANCE RANGE READY") && 
      SmartDashboard.getString("Angle Range Status: ", "NOT YET").equals("ANGLE RANGE READY") && 
      SmartDashboard.getString("Voltage Range Status: ", "NOT YET").equals("VOLTAGE RANGE READY")) {
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
    mySwerveMaster.stop();
    //myHookMaster.stop();
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
    mySwerveMaster.leftDownModule.turnMotor.set(driveController.getRightY());*/
    /*mySwerveMaster.rightUpModule.turnMotor.set(driveController.getLeftY());
    mySwerveMaster.rightDownModule.turnMotor.set(driveController.getRightY());*/
    //myArmMaster.launcherAngle.set(driveController.getLeftY());
    //myArmMaster.myIntake.groundRoller.set(driveController.getRightY());
    /*myArmMaster.myIntake.bottomIntake.set(driveController.getLeftY());
    myArmMaster.myIntake.topIntake.set(driveController.getRightY());*/
    /*myArmMaster.myLauncher.backMiddleRoller.set(driveController.getLeftY());
    myArmMaster.myLauncher.frontMiddleRoller.set(driveController.getRightY());*/
    /*myArmMaster.myLauncher.backLauncher.set(driveController.getLeftY());
    myArmMaster.myLauncher.frontLauncher.set(driveController.getRightY());*/
    //mySwerveMaster.leftDownModule.driveMotor.set(driveController.getLeftY());
   // mySwerveMaster.leftUpModule.driveMotor.set(Math.abs(driveController.getLeftY()) < Constants.driveControllerStopBelowThis ? 0 : driveController.getLeftY());

    /*if(driveController.getCrossButton()) {
      //testAngleMotor.setDirection(Relay.Direction.kForward);
      testAngleMotor.set(Relay.Value.kForward);
    } else if(driveController.getSquareButton()) {
      //testAngleMotor.setDirection(Relay.Direction.kReverse);
      testAngleMotor.set(Relay.Value.kOn);
    } else if(driveController.getTriangleButton()) {
      //testAngleMotor.setDirection(Relay.Direction.kBoth);
      testAngleMotor.set(Relay.Value.kOff);
    } else if(driveController.getCircleButton()) {
      //testAngleMotor.set(Relay.Value.kOn);
      testAngleMotor.set(Relay.Value.kReverse);
    }*/

    /*if(armController.getSquareButton()) {
      myHookMaster.leftHook.set(Relay.Value.kReverse);
    } else if(armController.getTriangleButton()) {
      myHookMaster.rightHook.set(Relay.Value.kReverse);
    } else if(armController.getCircleButton()) {
      myHookMaster.rightHook.set(Relay.Value.kForward);
    } else if(armController.getCrossButton()) {
      myHookMaster.leftHook.set(Relay.Value.kForward);
    } else {
      myHookMaster.leftHook.set(Relay.Value.kOff);
      myHookMaster.rightHook.set(Relay.Value.kOff);
    }*/

    SmartDashboard.putNumber("Drive Controller Left Y: ", driveController.getLeftY());
    SmartDashboard.putNumber("Drive Controller Right Y: ", driveController.getRightY());
  }

  public void shootNote() {
    while (!(SmartDashboard.getString("Launcher Status: ", "NOT YET").equals("LAUNCHER READY") &&  
      SmartDashboard.getString("Arm Angle Status: ", "NOT YET").equals("ARM ANGLE READY"))) {
      myArmMaster.updateVoltage();
      mySwerveMaster.updateRobotPosition(mySwerveMaster.getReducedAngle());
      mySwerveMaster.stop();
      myArmMaster.stopIntake();

      //Prep arm for scoring into speaker
      LauncherInfo info = myArmMaster.myLauncherInfoMaster.get(Math.sqrt(Math.pow(mySwerveMaster.getRobotPosition()[0] - Constants.gameConstants.speakerX, 2) + Math.pow(mySwerveMaster.getRobotPosition()[1] - (mySwerveMaster.blue ? Constants.gameConstants.blueConstants.speakerY : Constants.gameConstants.redConstants.speakerY), 2)), 0, myArmMaster.maxVoltage);
      //System.out.println(info.distance);
      myArmMaster.prepLauncher(info.speeds[0]);

      if(Math.abs(info.angles[0] - myArmMaster.getAngle()) < Constants.armConstants.angleTolerance) {
        SmartDashboard.putString("Arm Angle Status: ", "ARM ANGLE READY");
        myArmMaster.changeAngle(0);
      } else {
        SmartDashboard.putString("Arm Angle Status: ", "NOT YET");
        myArmMaster.changeAngle((info.angles[0] - myArmMaster.getAngle()) / 45);
      }
    }

    double time = Timer.getFPGATimestamp();

    while(Timer.getFPGATimestamp() - time < 0.1) {
      myArmMaster.updateVoltage();
      mySwerveMaster.updateRobotPosition(mySwerveMaster.getReducedAngle());
      myArmMaster.stopIntake();

      LauncherInfo info = myArmMaster.myLauncherInfoMaster.get(Math.sqrt(Math.pow(mySwerveMaster.getRobotPosition()[0] - Constants.gameConstants.speakerX, 2) + Math.pow(mySwerveMaster.getRobotPosition()[1] - (mySwerveMaster.blue ? Constants.gameConstants.blueConstants.speakerY : Constants.gameConstants.redConstants.speakerY), 2)), 0, myArmMaster.maxVoltage);
      myArmMaster.prepLauncher(info.speeds[0]);

      myArmMaster.fireLauncher();
    }
  }

  public void autoMoveFire(double[] pos) {
    boolean there = false;

    while(DriverStation.isAutonomous() && !there && 
      !(SmartDashboard.getString("Launcher Status: ", "NOT YET").equals("LAUNCHER READY") &&  
      SmartDashboard.getString("Arm Angle Status: ", "NOT YET").equals("ARM ANGLE READY"))) {
      myArmMaster.updateVoltage();
      mySwerveMaster.updateRobotPosition(mySwerveMaster.getReducedAngle());
      myArmMaster.stopIntake();
      LauncherInfo info = myArmMaster.myLauncherInfoMaster.get(Math.sqrt(Math.pow(mySwerveMaster.getRobotPosition()[0] - Constants.gameConstants.speakerX, 2) + Math.pow(mySwerveMaster.getRobotPosition()[1] - (mySwerveMaster.blue ? Constants.gameConstants.blueConstants.speakerY : Constants.gameConstants.redConstants.speakerY), 2)), 0, myArmMaster.maxVoltage);
      myArmMaster.prepLauncher(info.speeds[0]);

      if(Math.abs(info.angles[0] - myArmMaster.getAngle()) < Constants.armConstants.angleTolerance) {
        SmartDashboard.putString("Arm Angle Status: ", "ARM ANGLE READY");
        myArmMaster.changeAngle(0);
      } else {
        SmartDashboard.putString("Arm Angle Status: ", "NOT YET");
        myArmMaster.changeAngle((info.angles[0] - myArmMaster.getAngle()) / 22.5);
      }

      there = mySwerveMaster.driveTo(pos[0], pos[1], getDesiredAngle(pos[0], pos[1]));
    }

    double time = Timer.getFPGATimestamp();

    while(Timer.getFPGATimestamp() - time < 0.1) {
      myArmMaster.updateVoltage();
      mySwerveMaster.updateRobotPosition(mySwerveMaster.getReducedAngle());
      myArmMaster.stopIntake();

      LauncherInfo info = myArmMaster.myLauncherInfoMaster.get(Math.sqrt(Math.pow(mySwerveMaster.getRobotPosition()[0] - Constants.gameConstants.speakerX, 2) + Math.pow(mySwerveMaster.getRobotPosition()[1] - (mySwerveMaster.blue ? Constants.gameConstants.blueConstants.speakerY : Constants.gameConstants.redConstants.speakerY), 2)), 0, myArmMaster.maxVoltage);
      myArmMaster.prepLauncher(info.speeds[0]);

      myArmMaster.fireLauncher();
    }
  }

  public void autoMoveIntake(double[] pos) {
    double relativeX = mySwerveMaster.getRobotPosition()[0] - pos[0];
    double relativeY = mySwerveMaster.getRobotPosition()[1] - pos[1];
    double originalAngle = ((Math.atan2(relativeY, relativeX))) * 180 / Math.PI;

    boolean there = false;

    while(DriverStation.isAutonomous() && !there) {
      relativeX = mySwerveMaster.getRobotPosition()[0] - pos[0];
      relativeY = mySwerveMaster.getRobotPosition()[1] - pos[1];
      originalAngle = ((Math.atan2(relativeY, relativeX))) * 180 / Math.PI;

      myArmMaster.updateVoltage();
      mySwerveMaster.updateRobotPosition(mySwerveMaster.getReducedAngle());
      myArmMaster.intakeBottom();
      myArmMaster.stopLauncher();

      there = mySwerveMaster.driveTo(pos[0], pos[1], originalAngle);
    }

    double time = Timer.getFPGATimestamp();

    while(Timer.getFPGATimestamp() - time < 0.1) {
      myArmMaster.stopIntake();
    }
  }

  public void autoMovePrep(double[] pos) {
    boolean there = false;

    while(DriverStation.isAutonomous() && !there) {
      myArmMaster.updateVoltage();
      mySwerveMaster.updateRobotPosition(mySwerveMaster.getReducedAngle());
      myArmMaster.intakeBottom();
      LauncherInfo info = myArmMaster.myLauncherInfoMaster.get(Math.sqrt(Math.pow(mySwerveMaster.getRobotPosition()[0] - Constants.gameConstants.speakerX, 2) + Math.pow(mySwerveMaster.getRobotPosition()[1] - (mySwerveMaster.blue ? Constants.gameConstants.blueConstants.speakerY : Constants.gameConstants.redConstants.speakerY), 2)), 0, myArmMaster.maxVoltage);
      myArmMaster.prepLauncher(info.speeds[0]);
      myArmMaster.myLauncher.stopMiddle();

      if(Math.abs(info.angles[0] - myArmMaster.getAngle()) < Constants.armConstants.angleTolerance) {
        SmartDashboard.putString("Arm Angle Status: ", "ARM ANGLE READY");
        myArmMaster.changeAngle(0);
      } else {
        SmartDashboard.putString("Arm Angle Status: ", "NOT YET");
        myArmMaster.changeAngle((info.angles[0] - myArmMaster.getAngle()) / 22.5);
      }

      there = mySwerveMaster.driveTo(pos[0], pos[1], getDesiredAngle(pos[0], pos[1]));
    }
  }

  public void autoMove(double[] pos) {
    boolean there = false;

    while(DriverStation.isAutonomous() && !there) {
      myArmMaster.updateVoltage();
      mySwerveMaster.updateRobotPosition(mySwerveMaster.getReducedAngle());
      myArmMaster.stop();

      there = mySwerveMaster.driveTo(pos[0], pos[1], pos[2]);
    }
  }

  public double getDesiredAngle(double x, double y) {
    double relativeX = Constants.gameConstants.speakerX - x;
    double relativeY = (mySwerveMaster.blue ? Constants.gameConstants.blueConstants.speakerY : Constants.gameConstants.redConstants.speakerY) - y;
    double originalAngle = (Math.atan2(-relativeX, relativeY)) * 180 / Math.PI;
    return mySwerveMaster.blue ? Math.max(Math.min(Constants.gameConstants.blueConstants.maxSpeakerAngle, originalAngle), Constants.gameConstants.blueConstants.minSpeakerAngle) : Math.max(Math.min(Constants.gameConstants.redConstants.maxSpeakerAngle, originalAngle), Constants.gameConstants.redConstants.minSpeakerAngle);
  }
}