package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.armConstants;

public class Launcher {
    public CANSparkMax backMiddleRoller;
    public CANSparkMax frontMiddleRoller;
    public CANSparkMax backLauncher;
    public CANSparkMax frontLauncher;

    public Launcher() {
        backMiddleRoller = new CANSparkMax(armConstants.IDs.backMiddleRoller, MotorType.kBrushless);
        frontMiddleRoller = new CANSparkMax(armConstants.IDs.frontMiddleRoller, MotorType.kBrushless);
        backLauncher = new CANSparkMax(armConstants.IDs.backLauncher, MotorType.kBrushless);
        frontLauncher = new CANSparkMax(armConstants.IDs.frontLauncher, MotorType.kBrushless);
    }

    public void intake() {
        backMiddleRoller.set(armConstants.middleIntakeSpeed);
        frontMiddleRoller.set(-armConstants.middleIntakeSpeed);
        backLauncher.set(armConstants.launcherIntakeSpeed);
        frontLauncher.set(-armConstants.launcherIntakeSpeed);
    }

    public void stopMiddle() {
        backMiddleRoller.set(0);
        frontMiddleRoller.set(0);
    }

    public void stopLauncher() {
        SmartDashboard.putBoolean("LAUNCHER READY: ", false);
        backLauncher.set(0);
        frontLauncher.set(0);
    }

    public void stop() {
        stopMiddle();
        stopLauncher();
    }

    public void prepLauncher(double speed) {
        backLauncher.set(speed);
        frontLauncher.set(-speed);

        boolean backCorrect = Math.abs(backLauncher.get() - speed) < armConstants.launcherPrepTolerance;
        boolean frontCorrect = Math.abs(frontLauncher.get() + speed) < armConstants.launcherPrepTolerance;

        if(backCorrect && frontCorrect && speed != 0) {
            SmartDashboard.putBoolean("LAUNCHER READY: ", true);
        } else {
            SmartDashboard.putBoolean("LAUNCHER READY: ", false);
        }
    }

    public void fireLauncher() {
        backMiddleRoller.set(backLauncher.get());
        frontMiddleRoller.set(frontLauncher.get());
    }
}
