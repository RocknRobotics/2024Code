package frc.robot;

import com.revrobotics.CANSparkMax;

import java.util.ArrayList;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.armConstants;
import frc.robot.Constants.gameConstants;

public class ArmMaster {
    public CANSparkMax launcherAngle;
    public DutyCycleEncoder angleEncoder;
    public double angleEncoderOffset;
    public Intake myIntake;
    public Launcher myLauncher;
    public LauncherInfoMaster myLauncherInfoMaster;
    public boolean blue;
    //voltage, Timer.getFPGATimeStamp()
    public ArrayList<double[]> voltageList;
    //Max voltage in the list, make sure to min(max()) it with 11.3 and 12.7
    public int maxVoltage;

    public ArmMaster() {
        launcherAngle = new CANSparkMax(armConstants.IDs.launcherAngle, MotorType.kBrushless);
        angleEncoder = new DutyCycleEncoder(armConstants.IDs.angleEncoder);
        angleEncoderOffset = armConstants.angleEncoderOffset;
        myIntake = new Intake();
        myLauncher = new Launcher();
        myLauncherInfoMaster = new LauncherInfoMaster();
        blue = false;

        voltageList = new ArrayList<double[]>();
        maxVoltage = 113;

        launcherAngle.setInverted(armConstants.motorInversion.launcherAngle);

        /*SmartDashboard.putNumber("Info: Distance: ", 0d);
        SmartDashboard.putNumber("Info: Speed: ", 0d);
        SmartDashboard.putBoolean("Update Speaker Info: ", false);
        SmartDashboard.putBoolean("Update Amp Info: ", false);
        SmartDashboard.putBoolean("Update Trap Info: ", false);
        SmartDashboard.putBoolean("Save New Info: ", false);*/

        SmartDashboard.putString("Launcher Status: ", "NOT YET");
        SmartDashboard.putString("Arm Angle Status: ", "NOT YET");
        SmartDashboard.putString("Distance Range Status: ", "NOT YET");
        SmartDashboard.putString("Voltage Range Status: ", "NOT YET");
    }

    public void intakeBottom() {
        myIntake.intake();
    }

    public void outtakeBottom() {
        myIntake.outtake();
    }

    public void intakeLauncher(double speed) {
        myLauncher.intake(speed);
    }

    public void stopAngle() {
        launcherAngle.set(Math.abs(-launcherAngle.get()) > armConstants.motorAccelRates.launcherAngle ? Math.signum(-launcherAngle.get()) * armConstants.motorAccelRates.launcherAngle + launcherAngle.get() : 0);
    }
    
    public void stopIntake() {
        myIntake.stop();
    }

    public void stopLauncher() {
        myLauncher.stop();
    }

    public void stop() {
        stopAngle();
        stopIntake();
        stopLauncher();
    }

    public void changeAngle(double speed) {
        if(getAngle() >= 96 && speed > 0) {
            stopAngle();
        } else {
            launcherAngle.set(Math.abs(speed - launcherAngle.get()) > armConstants.motorAccelRates.launcherAngle ? Math.signum(speed - launcherAngle.get()) * armConstants.motorAccelRates.launcherAngle + launcherAngle.get() : speed);
        }
    }

    public void prepLauncher(double speed) {
        myLauncher.prepLauncher(speed);
    }

    public void fireLauncher() {
        myLauncher.fireLauncher();
    }

    public double getAngle() {
        double temp = (angleEncoder.getAbsolutePosition() * armConstants.angleConversionFactor - angleEncoderOffset);

        while(temp <= 0) {
            temp += 360;
        }
        while(temp > 360) {
            temp -= 360;
        }

        return temp;
    }

    public void update(PS4Controller armController, double currX, double currY) {
        updateVoltage();
        //Left joystick == angle
        //Right joystick == speed
        boolean hadRightInput = false;
        boolean autoInput = false;

        if(armController.getSquareButton()) {
            autoInput = true;
            //Prep arm for scoring into speaker
            /*System.out.println(currX + "---" + currY);
            System.out.println(gameConstants.speakerX + "---" + gameConstants.blueConstants.speakerY);
            System.out.println(Math.sqrt(Math.pow(currX - gameConstants.speakerX, 2) + Math.pow(currY - (blue ? gameConstants.blueConstants.speakerY : gameConstants.redConstants.speakerY), 2)));*/
            LauncherInfo info = myLauncherInfoMaster.get(Math.sqrt(Math.pow(currX - gameConstants.speakerX, 2) + Math.pow(currY - (blue ? gameConstants.blueConstants.speakerY : gameConstants.redConstants.speakerY), 2)), 0, maxVoltage);
            //LauncherInfo info = myLauncherInfoMaster.launcherInfoList.get(0);
            prepLauncher(info.speeds[0]);

            /*System.out.println(info.angles[0]);
            System.out.println(info.distance);
            System.out.println(info.voltage);*/

            if(Math.abs(info.angles[0] - getAngle()) < armConstants.angleTolerance) {
                SmartDashboard.putString("Arm Angle Status: ", "ARM ANGLE READY");
                changeAngle(0);
            } else {
                SmartDashboard.putString("Arm Angle Status: ", "NOT YET");
                changeAngle((info.angles[0] - getAngle()) / 45);
            }
        } else if(armController.getCrossButton()) {
            autoInput = true;
            //Prep arm for scoring into amp
            //LauncherInfo info = myLauncherInfoMaster.get(Math.sqrt(Math.pow(currX - gameConstants.ampX, 2) + Math.pow(currY - (blue ? gameConstants.blueConstants.ampY : gameConstants.redConstants.ampY), 2)), 1);
            prepLauncher(0.11);
            
            if(Math.abs(65.4 - getAngle()) < armConstants.angleTolerance) {
                SmartDashboard.putString("Arm Angle Status: ", "ARM ANGLE READY");
                changeAngle(0);
            } else {
                SmartDashboard.putString("Arm Angle Status: ", "NOT YET");
                changeAngle((65.4 - getAngle()) / 45);
            }

            /*autoInput = true;
            //Prep arm for scoring into amp/picking up from source/shooting from close up
            //LauncherInfo info = myLauncherInfoMaster.get(Math.sqrt(Math.pow(currX - gameConstants.ampX, 2) + Math.pow(currY - (blue ? gameConstants.blueConstants.ampY : gameConstants.redConstants.ampY), 2)), 1);
            prepLauncher(0.15);
            
            if(Math.abs(69d - getAngle()) < armConstants.angleTolerance) {
                SmartDashboard.putString("Arm Angle Status: ", "ARM ANGLE READY");
                changeAngle(0);
            } else {
                SmartDashboard.putString("Arm Angle Status: ", "NOT YET");
                changeAngle((69d - getAngle()) / 45);
            }*/
        } else if(armController.getCircleButton()) {
            autoInput = true;
            //Prep arm for intaking from source
            //intakeLauncher(0.15);
            myLauncher.prepLauncher(Constants.armConstants.launcherIntakeSpeed);
            myLauncher.setMiddle(Constants.armConstants.launcherIntakeSpeed);
            myIntake.specialIntake();

            if(Math.abs(73.1 - getAngle()) < armConstants.angleTolerance) {
                SmartDashboard.putString("Arm Angle Status: ", "ARM ANGLE READY");
                changeAngle(0);
            } else {
                SmartDashboard.putString("Arm Angle Status: ", "NOT YET");
                changeAngle((73.1 - getAngle()) / 45);
            }

            /*autoInput = true;
            //Prep arm for scoring from farther away
            //LauncherInfo info = myLauncherInfoMaster.get(Math.sqrt(Math.pow(currX - gameConstants.ampX, 2) + Math.pow(currY - (blue ? gameConstants.blueConstants.ampY : gameConstants.redConstants.ampY), 2)), 1);
            prepLauncher(0.39);
            
            if(Math.abs(45d - getAngle()) < armConstants.angleTolerance) {
                SmartDashboard.putString("Arm Angle Status: ", "ARM ANGLE READY");
                changeAngle(0);
            } else {
                SmartDashboard.putString("Arm Angle Status: ", "NOT YET");
                changeAngle((45d - getAngle()) / 45);
            }*/
        } else if(armController.getTriangleButton()) {
            //Prep arm for launching across the arena
            autoInput = true;
            /*intakeBottom();
            prepLauncher(0.64);
            fireLauncher();*/
            prepLauncher(Robot.fireFactor);

            if(Math.abs(65.8 - getAngle()) < armConstants.angleTolerance) {
                SmartDashboard.putString("Arm Angle Status: ", "ARM ANGLE READY");
                changeAngle(0);
            } else {
                SmartDashboard.putString("Arm Angle Status: ", "NOT YET");
                changeAngle((65.8 - getAngle()) / 45);
            }
        } else if(armController.getOptionsButton()) {
            //Prep arm for scoring into trap
            autoInput = true;
            prepLauncher(0.37);
            
            if(Math.abs(78 - getAngle()) < armConstants.angleTolerance) {
                SmartDashboard.putString("Arm Angle Status: ", "ARM ANGLE READY");
                changeAngle(0);
            } else {
                SmartDashboard.putString("Arm Angle Status: ", "NOT YET");
                changeAngle((78 - getAngle()) / 45);
            }
        } else {
            SmartDashboard.putString("Launcher Status: ", "NOT YET");
            SmartDashboard.putString("Arm Angle Status: ", "NOT YET");
            SmartDashboard.putString("Distance Range Status: ", "NOT YET");
            SmartDashboard.putString("Voltage Range Status: ", "NOT YET");
            
            //Manual control
            double leftInput = Math.abs(armController.getLeftY()) < Constants.armControllerStopBelowThis ? 0 : armController.getLeftY() * Robot.angleFactor;
            double rightInput = Math.abs(armController.getRightY()) < Constants.armControllerStopBelowThis ? 0 : Math.signum(armController.getRightY()) * Robot.fireFactor;

            if(rightInput < 0) {
                prepLauncher(-rightInput);
                hadRightInput = true;
            } else if(rightInput > 0) {
                intakeLauncher(rightInput);
                hadRightInput = true;
            }

            changeAngle(leftInput);
        }

        if(armController.getPOV() == 180) {
            intakeBottom();

            if(hadRightInput || autoInput) {
                fireLauncher();
            }
        } else if(armController.getPOV() == 270) {
            outtakeBottom();
            myLauncher.stopLauncher();
            myLauncher.setMiddle(Constants.armConstants.launcherIntakeSpeed);
        } else if(armController.getPOV() == 90) {
            intakeLauncher(Constants.armConstants.intakeIntakeSpeed * 2);
        } else if(armController.getPOV() == 135) {
            intakeBottom();
            intakeLauncher(Constants.armConstants.intakeIntakeSpeed * 2);
        } else if(armController.getPOV() == 0) {
            intakeBottom();
            fireLauncher();
            /*SmartDashboard.putNumber("Front Middle Current Speed: ", myLauncher.frontMiddleRoller.get());
            SmartDashboard.putNumber("Back Middle Current Speed: ", myLauncher.backMiddleRoller.get());
            SmartDashboard.putNumber("Front Launcher Current Speed: ", myLauncher.frontLauncher.get());
            SmartDashboard.putNumber("Back Launcher Current Speed: ", myLauncher.backLauncher.get());*/
        } else if(!hadRightInput && !autoInput) {
            stopLauncher();
            stopIntake();
        }

        /*double distance = SmartDashboard.getNumber("Info: Distance: ", 0d);
        double angle = getAngle();
        double speed = SmartDashboard.getNumber("Info: Speed: ", 0d);
        int voltage = (int) Math.round(RobotController.getBatteryVoltage() * 10);

        if(distance == 0 || speed == 0 || voltage == 0) {
            SmartDashboard.putBoolean("Saved Successfully: ", false);
        } else if(SmartDashboard.getBoolean("Update Speaker Info: ", false)) {
            myLauncherInfoMaster.storeSpeaker(voltage, distance, angle, speed);

            SmartDashboard.putBoolean("Update Speaker Info: ", false);
            SmartDashboard.putBoolean("Saved Successfully: ", true);
        } else if(SmartDashboard.getBoolean("Update Amp Info: ", false)) {
            myLauncherInfoMaster.storeAmp(voltage, distance, angle, speed);

            SmartDashboard.putBoolean("Update Amp Info: ", false);
            SmartDashboard.putBoolean("Saved Successfully: ", true);
        } else if(SmartDashboard.getBoolean("Update Trap Info: ", false)) {
            myLauncherInfoMaster.storeSpeaker(voltage, distance, angle, speed);

            SmartDashboard.putBoolean("Update Trap Info: ", false);
            SmartDashboard.putBoolean("Saved Successfully: ", true);
        }

        if(SmartDashboard.getBoolean("Save New Info: ", false)) {
            myLauncherInfoMaster.updateDataFile();

            SmartDashboard.putBoolean("Save New Info: ", false);
        }*/
    }

    public void updateVoltage() {
        int curr = (int) Math.round(RobotController.getBatteryVoltage() * 10);

        if(curr > maxVoltage) {
            maxVoltage = curr;
        }

        voltageList.add(new double[]{curr, Timer.getFPGATimestamp()});

        while(Timer.getFPGATimestamp() - voltageList.get(0)[1] >= 20) {
            if(voltageList.remove(0)[0] == maxVoltage) {
                maxVoltage = 113;

                for(int i = 0; i < voltageList.size() && maxVoltage < 127; i++) {
                    if(voltageList.get(i)[0] > maxVoltage) {
                        maxVoltage = (int) Math.round(voltageList.get(i)[0]);
                    }
                }

                if(maxVoltage > 127) {
                    maxVoltage = 127;
                }
            }
        }
    }
}