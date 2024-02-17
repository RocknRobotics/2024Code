package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.hookConstants;

public class HookMaster {
    public CANSparkMax leftHook;
    public CANSparkMax rightHook;
    public int count;
    public boolean extended;
    public boolean countFulfilled;

    public HookMaster() {
        leftHook = new CANSparkMax(hookConstants.leftHookID, MotorType.kBrushless);
        rightHook = new CANSparkMax(hookConstants.rightHookID, MotorType.kBrushless);
        count = 0;
        extended = false;
        countFulfilled = false;
    }

    //returns false while count < hookConstants.countsTillTop
    public boolean extend() {
        extended = true;
        leftHook.set(1);
        rightHook.set(1);
        count++;
        return count == hookConstants.countsTillTop;
    }

    //returns false while count < hookConstants.countsTillTop
    public boolean retract() {
        extended = false;
        leftHook.set(-1);
        rightHook.set(-1);
        count++;
        return count == hookConstants.countsTillTop;
    }

    public void stop() {
        leftHook.set(0);
        rightHook.set(0);
    }

    public void resetCount() {
        count = 0;
    }

    public void update(boolean shareButtonPressed) {
        if(shareButtonPressed) {
            resetCount();

            if(extended) {
                countFulfilled = retract();
            } else {
                countFulfilled = extend();
            }
        }

        if(extended) {
            if(!countFulfilled) {
                countFulfilled = extend();
            } else {
                stop();
            }
        } else {
            if(!countFulfilled) {
                countFulfilled = retract();
            } else {
                stop();
            }
        }
    }
}
