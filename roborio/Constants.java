package frc.robot;

public final class Constants {
    //The diamatre (in metres) of the wheels for driving
    public static final double driveWheelDiameter = 0.1016d;
    
    //Update rate of the accelerometer, in Hz (range 4-200)
    public static final byte accelerometerUpdateFrequency = 50;

    //The physical max speed the robot can move in metres/second
    public static final double maxTranslationalSpeed = 3.420296449459403d;

    //The physical max speed the robot can rotate in radians/second
    public static final double maxAngularSpeed = Math.PI * 2;

    //The port the drive controller is connected to
    public static final int driveControllerPort = 0;
    public static final int armControllerPort = 1;

    //The value of the driveController below which we will treat it as a zero instead
    public static final double driveControllerStopBelowThis = 0.15;
    public static final double armControllerStopBelowThis = 0.15;

    //Starting positions of the drive modules relative to the center of the robot in meters as [x, y]
    public static final double[] leftUpStartPos = new double[]{-0.3425d, 0.3425d};
    public static final double[] leftDownStartPos = new double[]{-0.3425d, -0.3425d};
    public static final double[] rightUpStartPos = new double[]{0.3425d, 0.3425d};
    public static final double[] rightDownStartPos = new double[]{0.3425d, -0.3425d};

    public static final double accelerometerOdometerTolerancePosition = 0.1;
    public static final double accelerometerOdometerToleranceVelocity = 1;

    public static final class autoConstants {
        //Times to backup at certain positions
        public static double backupTimeLeft = 3;        
        public static double backupTimeCenter = 3;
        public static double backupTimeRight = 3;

        //Speeds to backup at certain positions
        public static double backupSpeedLeft = 0.25;        
        public static double backupSpeedCenter = 0.25;
        public static double backupSpeedRight = 0.25;

        //Speeds to turn at certain positions
        public static double turnSpeed = 0.25;

        //Auto turn tolerance
        public static double turnTolerance = 0.5;

        //Start poses relative to origin for auto as [x, y, angle]
        public static double[] blueLeft = {3.663, 1, 315};
        public static double[] blueCenter = {0, 1.25, 0};//{2.663, 1.25, 0};
        public static double[] blueRight = {1.663, 1, 45};
        public static double[] redLeft = {3.663, 15.541, 135};
        public static double[] redCenter = {2.663, 15.291, 180};
        public static double[] redRight = {1.663, 15.541, 225};

    }

    public static final class motorConstants {
        public static final class turnConstants {
            public static final class motorAccelRates {
                //Ranges from 0 to 2
                //The max percentage acceleration allowed within a periodic tick
                public static final double leftUp = 0.2;
                public static final double leftDown = 0.2;
                public static final double rightUp = 0.2;
                public static final double rightDown = 0.2;
            }

            //PID controller position constant
            public static final double kP = 1.0;

            //Gear ratio between the turn motors and the module
            public static final double gearRatio = 1d / 1d;

            //The amount of radians per rotation of the turn motor (Size of "wheel" being rotated doesn't matter---One full rotation
            //of any size wheel equals 2 radians)
            public static final double radsPerRotation = gearRatio * 2 * Math.PI;
            public static final double degreesPerRotation = 360 * radsPerRotation / (2 * Math.PI);

            //IDs of turn motors
            public static final int leftUpID = 1;
            public static final int leftDownID = 7;
            public static final int rightUpID = 3;
            public static final int rightDownID = 5;

            //Invert turn wheels
            public static final boolean leftUpInvert = false;
            public static final boolean leftDownInvert = false;
            public static final boolean rightUpInvert = false;
            public static final boolean rightDownInvert = false;

            //IDs of turn encoders
            public static final int leftUpEncoderID = 0;
            public static final int leftDownEncoderID = 2;
            public static final int rightUpEncoderID = 1;
            public static final int rightDownEncoderID = 3;

            //Encoder offsets---the values of the encoders when the module is in the "0" position (facing forward)
            public static final double leftUpOffset = 123.9;
            public static final double leftDownOffset = 254.8;
            public static final double rightUpOffset = 305.2;
            public static final double rightDownOffset = 276.6;

            //Invert turn encoders
            public static final boolean leftUpEncoderInvert = true;
            public static final boolean leftDownEncoderInvert = true;
            public static final boolean rightUpEncoderInvert = true;
            public static final boolean rightDownEncoderInvert = true;
        }

        public static final class driveConstants {
            public static final class motorAccelRates {
                //Ranges from 0 to 2
                //The max percentage acceleration allowed within a periodic tick
                public static final double leftUp = 0.2;
                public static final double leftDown = 0.2;
                public static final double rightUp = 0.2;
                public static final double rightDown = 0.2;
            }

            //The gear ratio between the drive motors and the drive wheel---the amount of drive wheel rotations per motor rotation
            public static final double gearRatio = 1d / 6.12d;

            //The amount of metres traveled per rotation of the drive motor (Circumference of wheel * wheel rotation per motor rotation)
            public static final double metresPerRotation = gearRatio * driveWheelDiameter * Math.PI;

            //Units in metres/second, the velocity below which the swerve module motors will stop instead of going to a desired state
            public static final double stopBelowThisVelocity = 0.015d;
            
            //The physical max speed in rotations/second of the drive motors
            public static final double maxSpeed = 5700d; //5714.28369140625

            //IDs of drive motors
            public static final int leftUpID = 2;
            public static final int leftDownID = 8;
            public static final int rightUpID = 4;
            public static final int rightDownID = 6;

            //Invert drive wheels
            public static final boolean leftUpInvert = false;
            public static final boolean leftDownInvert = false;
            public static final boolean rightUpInvert = false;
            public static final boolean rightDownInvert = false;

            //The distance in metres between the left wheels and the right wheels
            public static final double leftToRightDistanceMetres = 0.685d;

            //Distance in metres between the front wheels and the back wheels
            public static final double upToDownDistanceMetres = 0.685d;

            //Wheel won't drive if the angle difference between target and current is too big
            public static final double angleTolerance = 60;
        }
    }

    public static final class OperatorConstants {
      public static final int kDriverControllerPort = 0;
      public static final int kArmControllerPort = 1;
    }

    public static final class armConstants {
        public static final class IDs {
            //Right intake is bottom intake
            //Left intake is top intake
            /*
            Right Intake   9
            Ground Roller   10
            Back Launcher   11
            Front Launcher   12
            Left Intake   13
            Launcher Angle   16
            Front Middle Roller   15
            Back Middle Roller   16
            Right Hook 17
            Left Hook 18
            */

            public static final int groundRoller = 10;
            public static final int bottomIntake = 13;//9;
            public static final int topIntake = 9;//13;
            public static final int backMiddleRoller = 16;
            public static final int frontMiddleRoller = 15;
            public static final int backLauncher = 11;
            public static final int frontLauncher = 12;
            public static final int launcherAngle = 14;

            public static final int angleEncoder = 0;
        }

        public static final class motorInversion {
            public static final boolean groundRoller = false;
            public static final boolean bottomIntake = false;
            public static final boolean topIntake = true;
            public static final boolean backMiddleRoller = false;
            public static final boolean frontMiddleRoller = false;
            public static final boolean backLauncher = true;
            public static final boolean frontLauncher = true;
            public static final boolean launcherAngle = true;
        }

        public static final class motorAccelRates {
            //Ranges from 0 to 2
            //The max percentage acceleration allowed within a periodic tick
            public static final double groundRoller = 0.1;
            public static final double bottomIntake = 0.1;
            public static final double topIntake = 0.1;
            public static final double backMiddleRoller = 0.1;
            public static final double frontMiddleRoller = 0.1;
            public static final double backLauncher = 0.2;
            public static final double frontLauncher = 0.2;
            public static final double launcherAngle = 0.2;
        }

        //Range from -1 to 1
        //speed < 0 should represent the bottom row spinning such that the note travels away from the launcher
        //speed > 0 should represent the bottom row spinning such that the note travels closer to the launcher
        public static final double intakeIntakeSpeed = 0.4;//0.25;
        public static final double middleIntakeSpeed = 0.15;//0.25;
        public static final double launcherIntakeSpeed = -0.3;//-0.5;

        //We want the returned angle to be 90 degrees when straight up
        //This should be in degrees ie after the conversion factor is applied
        public static final double angleEncoderOffset = 202.2;
        //360 degrees per one rotation of the absolute encoder?
        public static final double angleConversionFactor = 360d;

        public static final double launcherPrepTolerance = 0.05;
        public static final double angleTolerance = 0.5;
    }

    public static final class hookConstants {
        public static final class motorAccelRates {
            //Ranges from 0 to 2
            //The max percentage acceleration allowed within a periodic tick
            public static final double leftHook = 0.2;
            public static final double rightHook = 0.2;
        }

        public static final int leftHookID = 1;
        public static final int rightHookID = 0;

        public static final boolean leftHookInverted = false;
        public static final boolean rightHookInverted = false;

        public static final double extensionSpeed = 0.1;//1;

        //The length of time (in milliseconds) it takes for the hooks to fully extend
        public static final long extendTimeMillis = 17000;
    }

    public static final class gameConstants {
        //assumes southwest corner from blue alliance pov is (0, 0), with moving north +y and moving east +x
        public static final double speakerX = 2.663;
        public static final double ampX = 0.0;
        public static final double trapX1 = 4.093;
        public static final double trapX2 = 3.740;
        public static final double trapX3 = 4.445;

        public static final double minAmpAngle = 0.0;
        public static final double maxAmpAngle = 360.0;

        public static final class blueConstants {
            public static final double speakerY = 0.0;
            public static final double ampY = 1.84;
            public static final double trapY1 = 5.275;
            public static final double trapY2 = 4.664;
            public static final double trapY3 = 4.664;

            public static final double minSpeakerAngle = 0.0;
            public static final double maxSpeakerAngle = 360.0;
            public static final double minTrap1Angle = 0.0;
            public static final double maxTrap1Angle = 360.0;
            public static final double minTrap2Angle = 0.0;
            public static final double maxTrap2Angle = 360.0;
            public static final double minTrap3Angle = 0.0;
            public static final double maxTrap3Angle = 360.0;
        }
        public static final class redConstants {
            public static final double speakerY = 16.541;
            public static final double ampY = 14.701;
            public static final double trapY1 = 11.266;
            public static final double trapY2 = 11.877;
            public static final double trapY3 = 11.877;

            public static final double minSpeakerAngle = 0.0;
            public static final double maxSpeakerAngle = 360.0;
            public static final double minTrap1Angle = 0.0;
            public static final double maxTrap1Angle = 360.0;
            public static final double minTrap2Angle = 0.0;
            public static final double maxTrap2Angle = 360.0;
            public static final double minTrap3Angle = 0.0;
            public static final double maxTrap3Angle = 360.0;
        }
    }
}