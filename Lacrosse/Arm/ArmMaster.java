package frc.robot.Arm;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Robot;
import frc.robot.Values.Constants;
import frc.robot.Values.Determinables;

public class ArmMaster {
    public ArmAngler myArmAngler;
    public Intake myIntake;
    public Middle myMiddle;
    public Launcher myLauncher;

    public DoublePublisher armAngleGetPublisher;
    public DoublePublisher armAngleRPMPublisher;
    public DoublePublisher armAngleAbsoluteAnglePublisher;
    //Ground, bottom, top
    public DoubleArrayPublisher intakeGetsPublisher;
    public DoubleArrayPublisher intakeRPMsPublisher;
    //Back, top
    public DoubleArrayPublisher middleGetsPublisher;
    public DoubleArrayPublisher middleRPMsPublisher;
    //Back, top
    public DoubleArrayPublisher launcherGetsPublisher;
    public DoubleArrayPublisher launcherRPMsPublisher;

    public DoubleSubscriber armAngleSetSubscriber;
    public DoubleArraySubscriber intakeSetsSubscriber;
    public DoubleArraySubscriber middleSetsSubscriber;
    public DoubleArraySubscriber launcherSetsSubscriber;

    public ArmMaster(NetworkTableInstance inst) {
        myArmAngler = new ArmAngler();
        myIntake = new Intake();
        myMiddle = new Middle();
        myLauncher = new Launcher();

        armAngleGetPublisher = inst.getDoubleTopic("rio/ArmMaster/armAngleGet").publish();
        armAngleRPMPublisher = inst.getDoubleTopic("rio/ArmMaster/armAngleRPM").publish();
        armAngleAbsoluteAnglePublisher = inst.getDoubleTopic("rio/ArmMaster/armAngleAbsoluteAngle").publish();
        intakeGetsPublisher = inst.getDoubleArrayTopic("rio/ArmMaster/intakeGets").publish();
        intakeRPMsPublisher = inst.getDoubleArrayTopic("rio/ArmMaster/intakeRPMs").publish();
        middleGetsPublisher = inst.getDoubleArrayTopic("rio/ArmMaster/middleGets").publish();
        middleRPMsPublisher = inst.getDoubleArrayTopic("rio/ArmMaster/middleRPMs").publish();
        launcherGetsPublisher = inst.getDoubleArrayTopic("rio/ArmMaster/launcherGets").publish();
        launcherRPMsPublisher = inst.getDoubleArrayTopic("rio/ArmMaster/launcherRPMs").publish();

        armAngleSetSubscriber = inst.getDoubleTopic("jetson/ArmMaster/armAngleSet").subscribe(0d);
        intakeSetsSubscriber = inst.getDoubleArrayTopic("jetson/ArmMaster/intakeSets").subscribe(new double[]{0d, 0d, 0d});
        middleSetsSubscriber = inst.getDoubleArrayTopic("jetson/ArmMaster/middleSets").subscribe(new double[]{0d, 0d});
        launcherSetsSubscriber = inst.getDoubleArrayTopic("jetson/ArmMaster/launcherSets").subscribe(new double[]{0d, 0d});
    }

    public void intake() {
        myIntake.combinedLimitedSet(Constants.Presets.Intake.intaking);
        myMiddle.combinedLimitedSet(Constants.Presets.Middle.intaking);
        myLauncher.combinedLimitedSet(Constants.Presets.Launcher.intaking);
    }

    public void stop() {
        myArmAngler.stop();
        myIntake.stop();
        myMiddle.stop();
        myLauncher.stop();
    }

    public void teleopUpdate() {
        double angleInput = Math.abs(Determinables.Controllers.armController.getLeftY()) < Constants.Controllers.armControllerDeadband 
            ? 0d 
            : Determinables.Controllers.armController.getLeftY() * Robot.armAngleFactor;
        double launcherInput = Math.abs(Determinables.Controllers.armController.getRightY()) < Constants.Controllers.armControllerDeadband 
            ? 0d 
            : Determinables.Controllers.armController.getRightY() * Robot.armFireFactor;
        launcherInput *= -1d;

        myArmAngler.angleSet(angleInput);

        myLauncher.combinedLimitedSet(launcherInput);

        if(Determinables.Controllers.armController.getPOV() == 0) {
            myMiddle.combinedLimitedSet(launcherInput);
        } else if(Determinables.Controllers.armController.getPOV() == 180) {
            myIntake.combinedLimitedSet(Constants.Presets.Intake.intaking);

            if(launcherInput > 0d) {
                myMiddle.combinedLimitedSet(launcherInput);
            } else {
                myMiddle.backCustomSpark.limitedSet(launcherInput);
                myMiddle.frontCustomSpark.limitedSet(-launcherInput);
            }
        } else if(Determinables.Controllers.armController.getPOV() == 270) {
            myIntake.combinedLimitedSet(Constants.Presets.Intake.outtaking);
            myMiddle.combinedLimitedSet(Constants.Presets.Middle.outtaking);
            myLauncher.combinedLimitedSet(Constants.Presets.Launcher.outtaking);
        } else {
            myIntake.stop();
            myMiddle.stop();
        }
    }

    public void updateNetwork() {
        myArmAngler.limitedSet(armAngleSetSubscriber.get());
        myIntake.combinedLimitedSet(intakeSetsSubscriber.get());
        myMiddle.combinedLimitedSet(middleSetsSubscriber.get());
        myLauncher.combinedLimitedSet(launcherSetsSubscriber.get());

        armAngleGetPublisher.set(myArmAngler.armAngleCustomSpark.get());
        armAngleRPMPublisher.set(myArmAngler.armAngleRelativeEncoder.getVelocity());
        armAngleAbsoluteAnglePublisher.set(myArmAngler.getAbsoluteAngle());
        intakeGetsPublisher.set(new double[]{myIntake.groundCustomSpark.get(), myIntake.bottomCustomSpark.get(), myIntake.topCustomSpark.get()});
        intakeRPMsPublisher.set(new double[]{myIntake.groundRelativeEncoder.getVelocity(), myIntake.bottomRelativeEncoder.getVelocity(), myIntake.topRelativeEncoder.getVelocity()});
        middleGetsPublisher.set(new double[]{myMiddle.backCustomSpark.get(), myMiddle.frontCustomSpark.get()});
        middleRPMsPublisher.set(new double[]{myMiddle.backRelativeEncoder.getVelocity(), myMiddle.frontRelativeEncoder.getVelocity()});
        launcherGetsPublisher.set(new double[]{myLauncher.backCustomSpark.get(), myLauncher.frontCustomSpark.get()});
        launcherRPMsPublisher.set(new double[]{myLauncher.backRelativeEncoder.getVelocity(), myLauncher.frontRelativeEncoder.getVelocity()});
    }
}
