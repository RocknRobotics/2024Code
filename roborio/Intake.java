package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.armConstants;

public class Intake {
    public CANSparkMax groundRoller;
    public CANSparkMax bottomIntake;
    public CANSparkMax topIntake;

    public Intake() {
        groundRoller = new CANSparkMax(armConstants.IDs.groundRoller, MotorType.kBrushless);
        bottomIntake = new CANSparkMax(armConstants.IDs.bottomIntake, MotorType.kBrushless);
        topIntake = new CANSparkMax(armConstants.IDs.topIntake, MotorType.kBrushless);

        groundRoller.setInverted(armConstants.motorInversion.groundRoller);
        bottomIntake.setInverted(armConstants.motorInversion.bottomIntake);
        topIntake.setInverted(armConstants.motorInversion.topIntake);
    }

    public void intake() {
        groundRoller.set(Math.abs(armConstants.intakeIntakeSpeed - groundRoller.get()) > armConstants.motorAccelRates.groundRoller ? Math.signum(armConstants.intakeIntakeSpeed - groundRoller.get()) * armConstants.motorAccelRates.groundRoller : armConstants.intakeIntakeSpeed);
        bottomIntake.set(Math.abs(armConstants.intakeIntakeSpeed - bottomIntake.get()) > armConstants.motorAccelRates.bottomIntake ? Math.signum(armConstants.intakeIntakeSpeed - bottomIntake.get()) * armConstants.motorAccelRates.bottomIntake : armConstants.intakeIntakeSpeed);
        topIntake.set(Math.abs(-armConstants.intakeIntakeSpeed - topIntake.get()) > armConstants.motorAccelRates.topIntake ? Math.signum(-armConstants.intakeIntakeSpeed - topIntake.get()) * armConstants.motorAccelRates.topIntake : -armConstants.intakeIntakeSpeed);
    }

    public void stop() {
        groundRoller.set(Math.abs(-groundRoller.get()) > armConstants.motorAccelRates.groundRoller ? Math.signum(-groundRoller.get()) * armConstants.motorAccelRates.groundRoller : 0);
        bottomIntake.set(Math.abs(-bottomIntake.get()) > armConstants.motorAccelRates.bottomIntake ? Math.signum(-bottomIntake.get()) * armConstants.motorAccelRates.bottomIntake : 0);
        topIntake.set(Math.abs(-topIntake.get()) > armConstants.motorAccelRates.topIntake ? Math.signum(-topIntake.get()) * armConstants.motorAccelRates.topIntake : 0);
    }
}