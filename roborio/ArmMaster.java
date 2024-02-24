package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PS4Controller;
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

    public ArmMaster() {
        launcherAngle = new CANSparkMax(armConstants.IDs.launcherAngle, MotorType.kBrushless);
        angleEncoder = new DutyCycleEncoder(armConstants.IDs.angleEncoder);
        angleEncoderOffset = armConstants.angleEncoderOffset;
        myIntake = new Intake();
        myLauncher = new Launcher();
        myLauncherInfoMaster = new LauncherInfoMaster();
        blue = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;

        SmartDashboard.putNumber("Info: Distance: ", 0d);
        SmartDashboard.putNumber("Info: Speed: ", 0d);
        SmartDashboard.putBoolean("Update Speaker Info: ", false);
        SmartDashboard.putBoolean("Update Amp Info: ", false);
        SmartDashboard.putBoolean("Update Trap Info: ", false);
        SmartDashboard.putBoolean("Save New Info: ", false);

        SmartDashboard.putString("Launcher Status: ", "NOT YET");
        SmartDashboard.putString("Arm Angle Status: ", "NOT YET");
        SmartDashboard.putString("Distance Range Status: ", "NOT YET");
    }

    public void intake() {
        myIntake.intake();
        myLauncher.intake();
    }

    public void stopAngle() {
        launcherAngle.set(0);
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
        launcherAngle.set(speed);
    }

    public void prepLauncher(double speed) {
        myLauncher.prepLauncher(speed);
    }

    public void fireLauncher() {
        myLauncher.fireLauncher();
    }

    public double getAngle() {
        double temp = 90 - (angleEncoder.getAbsolutePosition() * armConstants.angleConversionFactor - angleEncoderOffset);

        while(temp <= 0) {
            temp += 360;
        }
        while(temp > 360) {
            temp -= 360;
        }

        return temp;
    }

    public void update(PS4Controller armController, double currX, double currY) {
        //Left joystick == angle
        //Right joystick == speed
        //r2 == fire
        //r1 == intake

        if(armController.getSquareButton()) {
            //Prep arm for scoring into speaker
            LauncherInfo info = myLauncherInfoMaster.get(Math.sqrt(Math.pow(currX - gameConstants.speakerX, 2) + Math.pow(currY - (blue ? gameConstants.blueConstants.speakerY : gameConstants.redConstants.speakerY), 2)));
            prepLauncher(info.speeds[0]);

            if(Math.abs(info.angles[0] - getAngle()) < armConstants.angleTolerance) {
                SmartDashboard.putString("Arm Angle Status: ", "ARM ANGLE READY");
                changeAngle(0);
            } else {
                SmartDashboard.putString("Arm Angle Status: ", "NOT YET");
                changeAngle( (info.angles[0] - getAngle()) / 45);
            }
        } else if(armController.getCrossButton()) {
            //Prep arm for scoring into amp
            LauncherInfo info = myLauncherInfoMaster.get(Math.sqrt(Math.pow(currX - gameConstants.ampX, 2) + Math.pow(currY - (blue ? gameConstants.blueConstants.ampY : gameConstants.redConstants.ampY), 2)));
            prepLauncher(info.speeds[1]);
            
            if(Math.abs(info.angles[1] - getAngle()) < armConstants.angleTolerance) {
                SmartDashboard.putString("Arm Angle Status: ", "ARM ANGLE READY");
                changeAngle(0);
            } else {
                SmartDashboard.putString("Arm Angle Status: ", "NOT YET");
                changeAngle( (info.angles[1] - getAngle()) / 45);
            }
        } else if(armController.getCircleButton()) {
            //Prep arm for scoring into trap1
            LauncherInfo info = myLauncherInfoMaster.get(Math.sqrt(Math.pow(currX - gameConstants.trapX1, 2) + Math.pow(currY - (blue ? gameConstants.blueConstants.trapY1 : gameConstants.redConstants.trapY1), 2)));
            prepLauncher(info.speeds[2]);

            if(Math.abs(info.angles[2] - getAngle()) < armConstants.angleTolerance) {
                SmartDashboard.putString("Arm Angle Status: ", "ARM ANGLE READY");
                changeAngle(0);
            } else {
                SmartDashboard.putString("Arm Angle Status: ", "NOT YET");
                changeAngle( (info.angles[2] - getAngle()) / 45);
            }
        } else if(armController.getTriangleButton()) {
            //Prep arm for scoring into trap2
            LauncherInfo info = myLauncherInfoMaster.get(Math.sqrt(Math.pow(currX - gameConstants.trapX2, 2) + Math.pow(currY - (blue ? gameConstants.blueConstants.trapY2 : gameConstants.redConstants.trapY2), 2)));
            prepLauncher(info.speeds[2]);

            if(Math.abs(info.angles[2] - getAngle()) < armConstants.angleTolerance) {
                SmartDashboard.putString("Arm Angle Status: ", "ARM ANGLE READY");
                changeAngle(0);
            } else {
                SmartDashboard.putString("Arm Angle Status: ", "NOT YET");
                changeAngle( (info.angles[2] - getAngle()) / 45);
            }
        } else if(armController.getOptionsButton()) {
            //Prep arm for scoring into trap3
            LauncherInfo info = myLauncherInfoMaster.get(Math.sqrt(Math.pow(currX - gameConstants.trapX3, 2) + Math.pow(currY - (blue ? gameConstants.blueConstants.trapY3 : gameConstants.redConstants.trapY3), 2)));
            prepLauncher(info.speeds[2]);
            
            if(Math.abs(info.angles[2] - getAngle()) < armConstants.angleTolerance) {
                SmartDashboard.putString("Arm Angle Status: ", "ARM ANGLE READY");
                changeAngle(0);
            } else {
                SmartDashboard.putString("Arm Angle Status: ", "NOT YET");
                changeAngle( (info.angles[2] - getAngle()) / 45);
            }
        } else {
            SmartDashboard.putString("Launcher Status: ", "NOT YET");
            SmartDashboard.putString("Arm Angle Status: ", "NOT YET");
            SmartDashboard.putString("Distance Range Status: ", "NOT YET");
            
            //Manual control
            double leftInput = Math.abs(armController.getLeftY()) < Constants.armControllerStopBelowThis ? 0 : armController.getLeftY() * Robot.angleFactor;
            double rightInput = Math.abs(armController.getRightY()) < Constants.armControllerStopBelowThis ? 0 : armController.getRightY() * Robot.fireFactor;

            myIntake.stop();
            myLauncher.stopMiddle();
            prepLauncher(rightInput);

            changeAngle(leftInput);
        }

        if(armController.getR1Button()) {
            intake();
        } else if(armController.getR2Button()) {
            fireLauncher();
        } 

        double distance = SmartDashboard.getNumber("Info: Distance: ", 0d);
        double angle = getAngle();
        double speed = SmartDashboard.getNumber("Info: Speed: ", 0d);

        if(SmartDashboard.getBoolean("Update Speaker Info: ", false)) {
            myLauncherInfoMaster.storeSpeaker(distance, angle, speed);

            SmartDashboard.putBoolean("Update Speaker Info: ", false);
        } else if(SmartDashboard.getBoolean("Update Amp Info: ", false)) {
            myLauncherInfoMaster.storeAmp(distance, angle, speed);

            SmartDashboard.putBoolean("Update Amp Info: ", false);
        } else if(SmartDashboard.getBoolean("Update Trap Info: ", false)) {
            myLauncherInfoMaster.storeSpeaker(distance, angle, speed);

            SmartDashboard.putBoolean("Update Trap Info: ", false);
        }

        if(SmartDashboard.getBoolean("Save New Info: ", false)) {
            myLauncherInfoMaster.updateDataFile();

            SmartDashboard.putBoolean("Save New Info: ", false);
        }
    }
}