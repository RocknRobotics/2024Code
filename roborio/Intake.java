package frc.robot;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.armConstants;

public class Intake {
    public CANSparkMax groundRoller;
    public CANSparkMax bottomIntake;
    public CANSparkMax topIntake;

    public Intake() {
        /*NetworkTableInstance inst = NetworkTableInstance.create();
        inst.startServer();
        SmartDashboard.setNetworkTableInstance(inst);*/
        
        groundRoller = new CANSparkMax(armConstants.IDs.groundRoller, MotorType.kBrushless);
        bottomIntake = new CANSparkMax(armConstants.IDs.bottomIntake, MotorType.kBrushless);
        topIntake = new CANSparkMax(armConstants.IDs.topIntake, MotorType.kBrushless);

        groundRoller.setInverted(armConstants.motorInversion.groundRoller);
        bottomIntake.setInverted(armConstants.motorInversion.bottomIntake);
        topIntake.setInverted(armConstants.motorInversion.topIntake);
    }

    public void intake() {
        /*groundRoller.set(speed);
        bottomIntake.set(speed);
        topIntake.set(-speed);*/
        groundRoller.set(Math.abs(armConstants.intakeIntakeSpeed - groundRoller.get()) > armConstants.motorAccelRates.groundRoller ? Math.signum(armConstants.intakeIntakeSpeed - groundRoller.get()) * armConstants.motorAccelRates.groundRoller  + groundRoller.get() : armConstants.intakeIntakeSpeed);
        bottomIntake.set(Math.abs(armConstants.intakeIntakeSpeed - bottomIntake.get()) > armConstants.motorAccelRates.bottomIntake ? Math.signum(armConstants.intakeIntakeSpeed - bottomIntake.get()) * armConstants.motorAccelRates.bottomIntake + bottomIntake.get() : armConstants.intakeIntakeSpeed);
        topIntake.set(Math.abs(-armConstants.intakeIntakeSpeed - topIntake.get()) > armConstants.motorAccelRates.topIntake ? Math.signum(-armConstants.intakeIntakeSpeed - topIntake.get()) * armConstants.motorAccelRates.topIntake + topIntake.get() : -armConstants.intakeIntakeSpeed);
    }

    public void outtake() {
        groundRoller.set(Math.abs(-armConstants.intakeIntakeSpeed - groundRoller.get()) > armConstants.motorAccelRates.groundRoller ? Math.signum(-armConstants.intakeIntakeSpeed - groundRoller.get()) * armConstants.motorAccelRates.groundRoller + groundRoller.get() : -armConstants.intakeIntakeSpeed);
        bottomIntake.set(Math.abs(-armConstants.intakeIntakeSpeed - bottomIntake.get()) > armConstants.motorAccelRates.bottomIntake ? Math.signum(-armConstants.intakeIntakeSpeed - bottomIntake.get()) * armConstants.motorAccelRates.bottomIntake + bottomIntake.get() : -armConstants.intakeIntakeSpeed);
        topIntake.set(Math.abs(armConstants.intakeIntakeSpeed - topIntake.get()) > armConstants.motorAccelRates.topIntake ? Math.signum(armConstants.intakeIntakeSpeed - topIntake.get()) * armConstants.motorAccelRates.topIntake + topIntake.get() : armConstants.intakeIntakeSpeed);
    }

    public void stop() {
        /*groundRoller.set(0);
        bottomIntake.set(0);
        topIntake.set(0);*/
        groundRoller.set(Math.abs(-groundRoller.get()) > armConstants.motorAccelRates.groundRoller ? Math.signum(-groundRoller.get()) * armConstants.motorAccelRates.groundRoller + groundRoller.get() : 0);
        bottomIntake.set(Math.abs(-bottomIntake.get()) > armConstants.motorAccelRates.bottomIntake ? Math.signum(-bottomIntake.get()) * armConstants.motorAccelRates.bottomIntake + bottomIntake.get() : 0);
        topIntake.set(Math.abs(-topIntake.get()) > armConstants.motorAccelRates.topIntake ? Math.signum(-topIntake.get()) * armConstants.motorAccelRates.topIntake + topIntake.get() : 0);
    }
}