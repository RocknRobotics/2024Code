package frc.robot.Swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Robot;
import frc.robot.Values.Constants;
import frc.robot.Values.Determinables;

public class SwerveMaster {
    public SwerveModule[] mySwerveModules;
    public Pigeon2 myPigeon;
    public double pigeonHeadingOffset;

    public double previousHeading;

    public PIDController continousController;

    //X, Y, and Heading of the robot
    public double[] XYH;

    //X, Y, and heading change using wheels and pigeon
    public DoubleArrayPublisher rioXYHPublisher;
    //Current values returned by the get of each wheel
    public DoubleArrayPublisher motorDriveGetsPublisher;
    public DoubleArrayPublisher motorTurnGetsPublisher;
    //Above but for RPM
    public DoubleArrayPublisher motorDriveRPMsPublisher;
    public DoubleArrayPublisher motorTurnRPMsPublisher;
    //The absolute position of each module
    public DoubleArrayPublisher moduleAbsoluteAnglesPublisher;

    //Gets the sets of the motors as prescribed by the jetson (the jetson accounts for the rpm formula)
    public DoubleArraySubscriber motorDriveSetsSubscriber;
    public DoubleArraySubscriber motorTurnSetsSubscriber;
    //The x, y, and heading of the robot as calculated by the jetson (in case it's needed for wheel stuff)
    public DoubleArraySubscriber jetsonXYHSubscriber;

    public SwerveMaster(NetworkTableInstance inst) {
        mySwerveModules[0] = new SwerveModule(Constants.IDs.leftUpDrive, Constants.IDs.leftUpTurn, Constants.IDs.leftUpEncoder, 
            Constants.Inversions.leftUpDrive, Constants.Inversions.leftUpTurn, Constants.Inversions.leftUpEncoder, 
            Constants.Offsets.leftUpOffset);
        mySwerveModules[1] = new SwerveModule(Constants.IDs.leftDownDrive, Constants.IDs.leftDownTurn, Constants.IDs.leftDownEncoder, 
            Constants.Inversions.leftDownDrive, Constants.Inversions.leftDownTurn, Constants.Inversions.leftDownEncoder, 
            Constants.Offsets.leftDownOffset);
        mySwerveModules[2] = new SwerveModule(Constants.IDs.rightUpDrive, Constants.IDs.rightUpTurn, Constants.IDs.rightUpEncoder, 
            Constants.Inversions.rightUpDrive, Constants.Inversions.rightUpTurn, Constants.Inversions.rightUpEncoder, 
            Constants.Offsets.rightUpOffset);
        mySwerveModules[3] = new SwerveModule(Constants.IDs.rightDownDrive, Constants.IDs.rightDownTurn, Constants.IDs.rightDownEncoder, 
            Constants.Inversions.rightDownDrive, Constants.Inversions.rightDownTurn, Constants.Inversions.rightDownEncoder, 
            Constants.Offsets.rightDownOffset);

        myPigeon = new Pigeon2(Constants.IDs.pigeon2, "rio");
        myPigeon.optimizeBusUtilization();
        myPigeon.setYaw(0d);

        pigeonHeadingOffset = 0d;
        previousHeading = 0d;

        continousController = new PIDController(1d, 0d, 0d);
        continousController.enableContinuousInput(0d, 360d);

        rioXYHPublisher = inst.getDoubleArrayTopic("rio/SwerveMaster/rioXYH").publish();
        motorDriveGetsPublisher = inst.getDoubleArrayTopic("rio/SwerveMaster/motorDriveGets").publish();
        motorTurnGetsPublisher = inst.getDoubleArrayTopic("rio/SwerveMaster/motorTurnGets").publish();
        motorDriveRPMsPublisher = inst.getDoubleArrayTopic("rio/SwerveMaster/motorDriveRPMs").publish();
        motorTurnRPMsPublisher = inst.getDoubleArrayTopic("rio/SwerveMaster/motorTurnRPMs").publish();
        moduleAbsoluteAnglesPublisher = inst.getDoubleArrayTopic("rio/SwerveMaster/moduleAbsoluteAngles").publish();

        motorDriveSetsSubscriber = inst.getDoubleArrayTopic("jetson/SwerveMaster/motorDriveSets").subscribe(new double[]{0d, 0d, 0d, 0d});
        motorTurnSetsSubscriber = inst.getDoubleArrayTopic("jetson/SwerveMaster/motorTurnSets").subscribe(new double[]{0d, 0d, 0d, 0d});
        jetsonXYHSubscriber = inst.getDoubleArrayTopic("jetson/XYH").subscribe(new double[]{0d, 0d, 0d});
    }

    public double getHeading() {
        double temp = -myPigeon.getAngle() - pigeonHeadingOffset;

        while(temp <= 0d) {
            temp += 360d;
        }
        while(temp > 360d) {
            temp -= 360d;
        }

        return temp;
    }

    public double getHeadingNoOffset() {
        double temp = -myPigeon.getAngle() - pigeonHeadingOffset;

        while(temp <= 0d) {
            temp += 360d;
        }
        while(temp > 360d) {
            temp -= 360d;
        }

        return temp;
    }

    public void resetHeading(double newHeading) {
        pigeonHeadingOffset = getHeadingNoOffset() - newHeading;

        while(pigeonHeadingOffset <= 0d) {
            pigeonHeadingOffset += 360d;
        }
        while(pigeonHeadingOffset > 360d) {
            pigeonHeadingOffset -= 360d;
        }
    }

    public void combinedLimitedSet(double[] driveSets, double[] turnSets) {
        for(int i = 0; i < 4; i++) {
            mySwerveModules[i].combinedLimitedSet(driveSets[i], turnSets[i]);
        }
    }

    public void combinedRPMSet(double[] driveRPM, double[] turnRPM) {
        for(int i = 0; i < 4; i++) {
            mySwerveModules[i].combinedRPMSet(driveRPM[i], turnRPM[i]);
        }
    }

    public void stop() {
        for(int i = 0; i < 4; i++) {
            mySwerveModules[i].stop();
        }
    }

    public void updateNetwork() {
        double[] tempXYH = jetsonXYHSubscriber.get();
        combinedLimitedSet(motorDriveSetsSubscriber.get(), motorTurnSetsSubscriber.get());
        double currAngle = getHeading();
        double[] currXY = getOdometryChange();

        rioXYHPublisher.set(new double[]{currXY[0], currXY[1], continousController.calculate(getHeading(), currAngle)});
        motorDriveGetsPublisher.set(new double[]{mySwerveModules[0].driveCustomSpark.get(), mySwerveModules[1].driveCustomSpark.get(), 
            mySwerveModules[2].driveCustomSpark.get(), mySwerveModules[3].driveCustomSpark.get()});
        motorDriveRPMsPublisher.set(new double[]{mySwerveModules[0].driveRelativeEncoder.getVelocity(), mySwerveModules[1].driveRelativeEncoder.getVelocity(), 
            mySwerveModules[2].driveRelativeEncoder.getVelocity(), mySwerveModules[3].driveRelativeEncoder.getVelocity()});
        motorDriveGetsPublisher.set(new double[]{mySwerveModules[0].turnCustomSpark.get(), mySwerveModules[1].turnCustomSpark.get(), 
            mySwerveModules[2].turnCustomSpark.get(), mySwerveModules[3].turnCustomSpark.get()});
        motorDriveRPMsPublisher.set(new double[]{mySwerveModules[0].turnRelativeEncoder.getVelocity(), mySwerveModules[1].turnRelativeEncoder.getVelocity(), 
            mySwerveModules[2].turnRelativeEncoder.getVelocity(), mySwerveModules[3].turnRelativeEncoder.getVelocity()});
        moduleAbsoluteAnglesPublisher.set(new double[]{mySwerveModules[0].getAbsoluteModuleAngle(), mySwerveModules[1].getAbsoluteModuleAngle(), 
            mySwerveModules[2].getAbsoluteModuleAngle(), mySwerveModules[3].getAbsoluteModuleAngle()});

        XYH[0] = tempXYH[0]; XYH[1] = tempXYH[1]; XYH[2] = tempXYH[2];
        resetHeading(XYH[2]);
        previousHeading = XYH[2];
    }

    //Should return the change in x, y, since the last time period as calculated via wheel odometry
    public double[] getOdometryChange() {
        double[] output = new double[2];

        return output;
    }

    public void alignModules(double angle) {
        mySwerveModules[0].alignPosition(XYH, XYH[2], Constants.Measurements.leftUpModulePos);
        mySwerveModules[1].alignPosition(XYH, XYH[2], Constants.Measurements.leftDownModulePos);
        mySwerveModules[2].alignPosition(XYH, XYH[2], Constants.Measurements.rightUpModulePos);
        mySwerveModules[3].alignPosition(XYH, XYH[2], Constants.Measurements.rightDownModulePos);
    }

    public void teleopUpdate() {
        double inputX = Math.abs(Determinables.Controllers.driveController.getLeftX()) < Constants.Controllers.driveControllerDeadband 
            ? 0d 
            : Determinables.Controllers.driveController.getLeftX() * Robot.swerveTranslationalFactor;
        double inputY = Math.abs(Determinables.Controllers.driveController.getLeftY()) < Constants.Controllers.driveControllerDeadband 
            ? 0d 
            : Determinables.Controllers.driveController.getLeftY() * Robot.swerveTranslationalFactor;
        double inputH = Math.abs(Determinables.Controllers.driveController.getRightX()) < Constants.Controllers.driveControllerDeadband 
            ? 0d 
            : Determinables.Controllers.driveController.getRightX() * Robot.swerveRotationalFactor;

        double adjustedX = inputX / (Math.abs(inputY) + Math.abs(inputH) + 1);
        double adjustedY = inputY / (Math.abs(inputX) + Math.abs(inputH) + 1);
        double adjustedH = inputH / (Math.abs(inputX) + Math.abs(inputY) + 1);

        double currentHeading = getHeading();

        for(int i = 0; i < 4; i++) {
            double translationalAngle = Math.atan2(-adjustedY, adjustedX) + 3 * Math.PI / 2;
            translationalAngle = translationalAngle - currentHeading * Math.PI / 180;

            while(translationalAngle <= 0) {
                translationalAngle += 2 * Math.PI;
            }
            while(translationalAngle > 2 * Math.PI) {
                translationalAngle -= 2 * Math.PI;
            }

            double translationalMagnitude = Math.sqrt(Math.pow(adjustedX, 2) + Math.pow(adjustedY, 2));

            double rotationalMagnitude = Math.abs(adjustedH);

            double[] modulePos = mySwerveModules[i].getModulePosition();
            double rotationalAngle = Math.atan2(modulePos[1], modulePos[0]);

            double tempX = translationalMagnitude * Math.cos(translationalAngle) 
                + rotationalMagnitude * Math.cos(rotationalAngle);
            double tempY = translationalMagnitude * Math.sin(translationalAngle) 
                + rotationalMagnitude * Math.sin(rotationalAngle);

            double desiredAngle = Math.atan2(tempX, tempY) * 180 / Math.PI;
            double desiredMagnitude = Math.sqrt(Math.pow(tempX, 2) + Math.pow(tempY, 2));

            while(desiredAngle <= 0) {
                desiredAngle += 360;
            }
            while(desiredAngle > 360) {
                desiredAngle -= 360;
            }

            double anglePlus = desiredAngle + 180;
            double angleMinus = desiredAngle - 180;

            double moduleAbsoluteAngle = mySwerveModules[i].getAbsoluteModuleAngle();

            double desiredTurnSet = Math.abs(continousController.calculate(moduleAbsoluteAngle, desiredAngle)) / 90d;
            double desiredPlusSet = Math.abs(continousController.calculate(moduleAbsoluteAngle, anglePlus)) / 90d;
            double desiredMinusSet = Math.abs(continousController.calculate(moduleAbsoluteAngle, angleMinus)) / 90d;

            if(Math.abs(desiredTurnSet) > Math.abs(desiredPlusSet)) {
                mySwerveModules[i].combinedLimitedSet(-desiredMagnitude, desiredPlusSet);
            } else if(Math.abs(desiredTurnSet) > Math.abs(desiredMinusSet)) {
                mySwerveModules[i].combinedLimitedSet(-desiredMagnitude, desiredMinusSet);
            } else {
                mySwerveModules[i].combinedLimitedSet(desiredMagnitude, desiredTurnSet);
            }
        }
    }
}
