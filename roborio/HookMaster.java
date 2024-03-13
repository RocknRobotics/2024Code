package frc.robot;

import edu.wpi.first.wpilibj.Relay;
import frc.robot.Constants.hookConstants;

public class HookMaster {
    public Relay leftHook;
    public Relay rightHook;
    public long startMillis; //BEWARE: Will not work in approximately 292471154.678 years (not accounting for leap years)
    public boolean extended;
    public boolean timeFulfilled;

    public HookMaster() {
        /*NetworkTableInstance inst = NetworkTableInstance.create();
        inst.startServer();
        SmartDashboard.setNetworkTableInstance(inst);*/
        
        leftHook = new Relay(hookConstants.leftHookID, Relay.Direction.kBoth);
        rightHook = new Relay(hookConstants.rightHookID, Relay.Direction.kBoth);

        startMillis = System.currentTimeMillis();
        extended = false;
        timeFulfilled = false;
        startMillis = Long.MIN_VALUE / 2;
    }

    //returns false while count < hookConstants.countsTillTop
    public void extend() {
        leftHook.set(Relay.Value.kReverse); //Might need to switch forward and reverse
        rightHook.set(Relay.Value.kReverse);
    }

    //returns false while count < hookConstants.countsTillTop
    public void retract() {
        leftHook.set(Relay.Value.kForward);
        rightHook.set(Relay.Value.kForward);
    }

    public void stop() {
        leftHook.set(Relay.Value.kOff);
        rightHook.set(Relay.Value.kOff);
    }

    public void update(int drivePov) {
        if(drivePov == 0) {
            extend();
        } else if(drivePov == 180) {
            retract();
        } else {
            stop();
        }
        /*if(shareButtonPressed) {
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
        }*/
    }
}