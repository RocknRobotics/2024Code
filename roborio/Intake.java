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
    }

    public void intake() {
        groundRoller.set(armConstants.intakeIntakeSpeed);
        bottomIntake.set(armConstants.intakeIntakeSpeed);
        topIntake.set(-armConstants.intakeIntakeSpeed);
    }

    public void stop() {
        groundRoller.set(0);
        bottomIntake.set(0);
        topIntake.set(0);
    }
}
