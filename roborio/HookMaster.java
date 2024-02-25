package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.hookConstants;

public class HookMaster {
    public CANSparkMax leftHook;
    public CANSparkMax rightHook;
    public long startMillis; //BEWARE: Will not work in approximately 292471154.678 years (not accounting for leap years)
    public boolean extended;
    public boolean timeFulfilled;

    public HookMaster() {
        leftHook = new CANSparkMax(hookConstants.leftHookID, MotorType.kBrushless);
        rightHook = new CANSparkMax(hookConstants.rightHookID, MotorType.kBrushless);

        leftHook.setInverted(hookConstants.leftHookInverted);
        rightHook.setInverted(hookConstants.rightHookInverted);

        startMillis = System.currentTimeMillis();
        extended = false;
        timeFulfilled = false;
    }

    //returns false while count < hookConstants.countsTillTop
    public void extend() {
        leftHook.set(Math.abs(hookConstants.extensionSpeed - leftHook.get()) > hookConstants.motorAccelRates.leftHook ? Math.signum(hookConstants.extensionSpeed - leftHook.get()) * hookConstants.motorAccelRates.leftHook : hookConstants.extensionSpeed);
        rightHook.set(Math.abs(hookConstants.extensionSpeed - rightHook.get()) > hookConstants.motorAccelRates.rightHook ? Math.signum(hookConstants.extensionSpeed - rightHook.get()) * hookConstants.motorAccelRates.rightHook : hookConstants.extensionSpeed);
    }

    //returns false while count < hookConstants.countsTillTop
    public void retract() {
        leftHook.set(Math.abs(-hookConstants.extensionSpeed - leftHook.get()) > hookConstants.motorAccelRates.leftHook ? Math.signum(-hookConstants.extensionSpeed - leftHook.get()) * hookConstants.motorAccelRates.leftHook : -hookConstants.extensionSpeed);
        rightHook.set(Math.abs(-hookConstants.extensionSpeed - rightHook.get()) > hookConstants.motorAccelRates.rightHook ? Math.signum(-hookConstants.extensionSpeed - rightHook.get()) * hookConstants.motorAccelRates.rightHook : -hookConstants.extensionSpeed);
    }

    public void stop() {
        leftHook.set(Math.abs(-leftHook.get()) > hookConstants.motorAccelRates.leftHook ? Math.signum(-leftHook.get()) * hookConstants.motorAccelRates.leftHook : 0);
        rightHook.set(Math.abs(-rightHook.get()) > hookConstants.motorAccelRates.rightHook ? Math.signum(-rightHook.get()) * hookConstants.motorAccelRates.rightHook : 0);
    }

    public void update(boolean shareButtonPressed) {
        if(shareButtonPressed) {
            startMillis = System.currentTimeMillis();

            if(extended) {
                retract();
                extended = false;
            } else {
                extend();
                extended = true;
            }
        }

        long duration = System.currentTimeMillis() - startMillis;

        if(duration >= Constants.hookConstants.extendTimeMillis) {
            stop();
        } else {
            if(extended) {
                extend();
            } else {
                retract();
            }
        }
    }
}