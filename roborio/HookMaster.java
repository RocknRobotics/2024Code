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
        startMillis = System.currentTimeMillis();
        extended = false;
        timeFulfilled = false;
    }

    //returns false while count < hookConstants.countsTillTop
    public void extend() {
        leftHook.set(1);
        rightHook.set(1);
    }

    //returns false while count < hookConstants.countsTillTop
    public void retract() {
        leftHook.set(-1);
        rightHook.set(-1);
    }

    public void stop() {
        leftHook.set(0);
        rightHook.set(0);
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
        }
    }
}
