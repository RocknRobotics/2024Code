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
        /*NetworkTableInstance inst = NetworkTableInstance.create();
        inst.startServer();
        SmartDashboard.setNetworkTableInstance(inst);*/
        
        backMiddleRoller = new CANSparkMax(armConstants.IDs.backMiddleRoller, MotorType.kBrushless);
        frontMiddleRoller = new CANSparkMax(armConstants.IDs.frontMiddleRoller, MotorType.kBrushless);
        backLauncher = new CANSparkMax(armConstants.IDs.backLauncher, MotorType.kBrushless);
        frontLauncher = new CANSparkMax(armConstants.IDs.frontLauncher, MotorType.kBrushless);

        backMiddleRoller.setInverted(armConstants.motorInversion.backMiddleRoller);
        frontMiddleRoller.setInverted(armConstants.motorInversion.frontMiddleRoller);
        backLauncher.setInverted(armConstants.motorInversion.backLauncher);
        frontLauncher.setInverted(armConstants.motorInversion.frontLauncher);
    }

    public void intake(double speed) {
        speed *= 0.5;
        /*backMiddleRoller.set(armConstants.middleIntakeSpeed);
        frontMiddleRoller.set(-armConstants.middleIntakeSpeed);
        backLauncher.set(armConstants.launcherIntakeSpeed);
        frontLauncher.set(-armConstants.launcherIntakeSpeed);*/
        backMiddleRoller.set(Math.abs(speed - backMiddleRoller.get()) > armConstants.motorAccelRates.backMiddleRoller ? Math.signum(speed - backMiddleRoller.get()) * armConstants.motorAccelRates.backMiddleRoller + backMiddleRoller.get() : speed);
        frontMiddleRoller.set(Math.abs(-speed - frontMiddleRoller.get()) > armConstants.motorAccelRates.frontMiddleRoller ? Math.signum(-speed - frontMiddleRoller.get()) * armConstants.motorAccelRates.frontMiddleRoller + frontMiddleRoller.get() : -speed);
        backLauncher.set(Math.abs(2 * -speed - backLauncher.get()) > armConstants.motorAccelRates.backLauncher ? Math.signum(2 * -speed - backLauncher.get()) * armConstants.motorAccelRates.backLauncher + backLauncher.get() : 2 * -speed);
        frontLauncher.set(-Math.abs(2 * speed - frontLauncher.get()) > armConstants.motorAccelRates.frontLauncher ? Math.signum(2 * speed - frontLauncher.get()) * armConstants.motorAccelRates.frontLauncher + frontLauncher.get() : 2 * speed);
    }

    public void stopMiddle() {
        /*backMiddleRoller.set(0);
        frontMiddleRoller.set(0);*/
        backMiddleRoller.set(Math.abs(backMiddleRoller.get()) > armConstants.motorAccelRates.backMiddleRoller ? Math.signum(-backMiddleRoller.get()) * armConstants.motorAccelRates.backMiddleRoller + backMiddleRoller.get() : 0);
        frontMiddleRoller.set(Math.abs(frontMiddleRoller.get()) > armConstants.motorAccelRates.frontMiddleRoller ? Math.signum(-frontMiddleRoller.get()) * armConstants.motorAccelRates.frontMiddleRoller + frontMiddleRoller.get() : 0);
    }

    public void stopLauncher() {
        /*backLauncher.set(0);
        frontLauncher.set(0);*/
        SmartDashboard.putString("Launcher Status: ", "NOT YET");
        backLauncher.set(Math.abs(-backLauncher.get()) > armConstants.motorAccelRates.backLauncher ? Math.signum(-backLauncher.get()) * armConstants.motorAccelRates.backLauncher + backLauncher.get() : 0);
        frontLauncher.set(-Math.abs(-frontLauncher.get()) > armConstants.motorAccelRates.frontLauncher ? Math.signum(-frontLauncher.get()) * armConstants.motorAccelRates.frontLauncher + frontLauncher.get() : 0);
    }

    public void stop() {
        stopMiddle();
        stopLauncher();
    }

    public void prepLauncher(double speed) {
        backLauncher.set(Math.abs(speed - backLauncher.get()) > armConstants.motorAccelRates.backLauncher ? Math.signum(speed - backLauncher.get()) * armConstants.motorAccelRates.backLauncher + backLauncher.get() : speed);
        frontLauncher.set(Math.abs(-speed - frontLauncher.get()) > armConstants.motorAccelRates.frontLauncher ? Math.signum(-speed - frontLauncher.get()) * armConstants.motorAccelRates.frontLauncher + frontLauncher.get() : -speed);
        //backLauncher.set(speed);
        //frontLauncher.set(-speed);

        boolean backCorrect = Math.abs(backLauncher.get() - speed) < armConstants.launcherPrepTolerance;
        boolean frontCorrect = Math.abs(frontLauncher.get() + speed) < armConstants.launcherPrepTolerance;

        if(backCorrect && frontCorrect && speed != 0) {
            SmartDashboard.putString("Launcher Status: ", "LAUNCHER READY");
        } else {
            SmartDashboard.putString("Launcher Status: ", "NOT YET");
        }
    }

    public void fireLauncher() {
        backMiddleRoller.set(Math.abs(backLauncher.get() - backMiddleRoller.get()) > armConstants.motorAccelRates.backMiddleRoller ? Math.signum(backLauncher.get() - backMiddleRoller.get()) * armConstants.motorAccelRates.backMiddleRoller + backMiddleRoller.get() : backLauncher.get());
        frontMiddleRoller.set(Math.abs(frontLauncher.get() - frontMiddleRoller.get()) > armConstants.motorAccelRates.frontMiddleRoller ? Math.signum(frontLauncher.get() - frontMiddleRoller.get()) * armConstants.motorAccelRates.frontMiddleRoller + frontMiddleRoller.get() : frontLauncher.get());
    }
}