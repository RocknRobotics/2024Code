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

        backMiddleRoller.setInverted(armConstants.motorInversion.backMiddleRoller);
        frontMiddleRoller.setInverted(armConstants.motorInversion.frontMiddleRoller);
        backLauncher.setInverted(armConstants.motorInversion.backLauncher);
        frontLauncher.setInverted(armConstants.motorInversion.frontLauncher);
    }

    public void intake() {
        backMiddleRoller.set(Math.abs(armConstants.middleIntakeSpeed - backMiddleRoller.get()) > armConstants.motorAccelRates.backMiddleRoller ? Math.signum(armConstants.middleIntakeSpeed - backMiddleRoller.get()) * armConstants.motorAccelRates.backMiddleRoller : armConstants.middleIntakeSpeed);
        frontMiddleRoller.set(Math.abs(-armConstants.middleIntakeSpeed - frontMiddleRoller.get()) > armConstants.motorAccelRates.frontMiddleRoller ? Math.signum(-armConstants.middleIntakeSpeed - frontMiddleRoller.get()) * armConstants.motorAccelRates.frontMiddleRoller : -armConstants.middleIntakeSpeed);
        backLauncher.set(Math.abs(armConstants.launcherIntakeSpeed - backLauncher.get()) > armConstants.motorAccelRates.backLauncher ? Math.signum(armConstants.launcherIntakeSpeed - backLauncher.get()) * armConstants.motorAccelRates.backLauncher : armConstants.launcherIntakeSpeed);
        frontLauncher.set(-Math.abs(-armConstants.launcherIntakeSpeed - frontLauncher.get()) > armConstants.motorAccelRates.frontLauncher ? Math.signum(-armConstants.launcherIntakeSpeed - frontLauncher.get()) * armConstants.motorAccelRates.frontLauncher : -armConstants.launcherIntakeSpeed);
    }

    public void stopMiddle() {
        backMiddleRoller.set(Math.abs(backMiddleRoller.get()) > armConstants.motorAccelRates.backMiddleRoller ? Math.signum(-backMiddleRoller.get()) * armConstants.motorAccelRates.backMiddleRoller : 0);
        frontMiddleRoller.set(Math.abs(frontMiddleRoller.get()) > armConstants.motorAccelRates.frontMiddleRoller ? Math.signum(-frontMiddleRoller.get()) * armConstants.motorAccelRates.frontMiddleRoller : 0);
    }

    public void stopLauncher() {
        SmartDashboard.putString("Launcher Status: ", "NOT YET");
        backLauncher.set(Math.abs(-backLauncher.get()) > armConstants.motorAccelRates.backLauncher ? Math.signum(-backLauncher.get()) * armConstants.motorAccelRates.backLauncher : 0);
        frontLauncher.set(-Math.abs(-frontLauncher.get()) > armConstants.motorAccelRates.frontLauncher ? Math.signum(-frontLauncher.get()) * armConstants.motorAccelRates.frontLauncher : 0);
    }

    public void stop() {
        stopMiddle();
        stopLauncher();
    }

    public void prepLauncher(double speed) {
        backLauncher.set(Math.abs(speed - backLauncher.get()) > armConstants.motorAccelRates.backLauncher ? Math.signum(speed - backLauncher.get()) * armConstants.motorAccelRates.backLauncher : speed);
        frontLauncher.set(-Math.abs(-speed - frontLauncher.get()) > armConstants.motorAccelRates.frontLauncher ? Math.signum(-speed - frontLauncher.get()) * armConstants.motorAccelRates.frontLauncher : -speed);

        boolean backCorrect = Math.abs(backLauncher.get() - speed) < armConstants.launcherPrepTolerance;
        boolean frontCorrect = Math.abs(frontLauncher.get() + speed) < armConstants.launcherPrepTolerance;

        if(backCorrect && frontCorrect && speed != 0) {
            SmartDashboard.putString("Launcher Status: ", "LAUNCHER READY");
        } else {
            SmartDashboard.putString("Launcher Status: ", "NOT YET");
        }
    }

    public void fireLauncher() {
        backMiddleRoller.set(Math.abs(backLauncher.get() - backMiddleRoller.get()) > armConstants.motorAccelRates.backMiddleRoller ? Math.signum(backLauncher.get() - backMiddleRoller.get()) * armConstants.motorAccelRates.backMiddleRoller : backLauncher.get());
        frontMiddleRoller.set(Math.abs(frontLauncher.get() - frontMiddleRoller.get()) > armConstants.motorAccelRates.frontMiddleRoller ? Math.signum(frontLauncher.get() - frontMiddleRoller.get()) * armConstants.motorAccelRates.frontMiddleRoller : frontLauncher.get());
    }
}