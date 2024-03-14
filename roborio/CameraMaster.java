package frc.robot;

import java.util.concurrent.atomic.AtomicBoolean;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.Timer;

public class CameraMaster {
    public UsbCamera frontCamera;
    public CvSink frontSink;
    public Mat frontImage;
    public Mat grayImage;
    public AprilTagDetector onlyDetector;
    public AprilTagPoseEstimator onlyEstimator;

    public Pose3d[] tagWorldTransforms = new Pose3d[]{
        new Pose3d(7.9583, 15.1176, 1.3559, new Rotation3d(0, 0, 120 * Math.PI / 180)), 
        new Pose3d(7.3205, 16.2232, 1.3559, new Rotation3d(0, 0, 120 * Math.PI / 180)), 
        new Pose3d(3.2215, 16.6174, 1.4511, new Rotation3d(0, 0, 180 * Math.PI / 180)), 
        new Pose3d(2.6563, 16.6174, 1.4511, new Rotation3d(0, 0, 180 * Math.PI / 180)), 
        new Pose3d(     0, 14.7389, 1.3559, new Rotation3d(0, 0, 270 * Math.PI / 180)), 
        new Pose3d(     0,  1.8796, 1.3559, new Rotation3d(0, 0, 270 * Math.PI / 180)), 
        new Pose3d(2.6563,       0, 1.4511, new Rotation3d(0, 0,   0 * Math.PI / 180)), 
        new Pose3d(3.2215,       0, 1.4511, new Rotation3d(0, 0,   0 * Math.PI / 180)), 
        new Pose3d(7.3205,  0.3942, 1.3559, new Rotation3d(0, 0,  60 * Math.PI / 180)), 
        new Pose3d(7.9583,  1.4996, 1.3559, new Rotation3d(0, 0,  60 * Math.PI / 180)), 
        new Pose3d(4.4910, 11.9428, 1.3208, new Rotation3d(0, 0, 300 * Math.PI / 180)), 
        new Pose3d(3.7059, 11.9428, 1.3208, new Rotation3d(0, 0,  60 * Math.PI / 180)), 
        new Pose3d(4.0991, 11.2583, 1.3208, new Rotation3d(0, 0, 180 * Math.PI / 180)), 
        new Pose3d(4.0991,  5.3589, 1.3208, new Rotation3d(0, 0,   0 * Math.PI / 180)), 
        new Pose3d(3.7059,  4.6794, 1.3208, new Rotation3d(0, 0, 120 * Math.PI / 180)), 
        new Pose3d(4.4910,  4.6794, 1.3208, new Rotation3d(0, 0, 240 * Math.PI / 180))};

    //Camera is 45 degrees above the y axis
    public Transform3d frontCameraTransform3d = new Transform3d(-0.27, -0.33, 0, new Rotation3d(0, 0, 0));
    public double cameraAngle = 45 * Math.PI / 180;

    public static AtomicBoolean cameraPoseLock;
    public static boolean updatedPose;
    public static double updatedX;
    public static double updatedY;
    public static double updatedHeading;
    public static double updateTime;

    public CameraMaster() {
        int width = 640;
        int height = 360;
        frontCamera = new UsbCamera("Front Camera", 0);
        frontCamera.setVideoMode(PixelFormat.kMJPEG, width, height, 2);
        frontSink = new CvSink("Front Sink", PixelFormat.kBGR);
        frontSink.setSource(frontCamera);
        frontImage = new Mat(width, height, CvType.CV_8UC3, new Scalar(0, 0, 0));
        grayImage = new Mat(width, height, CvType.CV_8UC1, new Scalar(0));

        onlyDetector = new AprilTagDetector();
        onlyDetector.addFamily("tag36h11", 0);

        double diagonal = Math.sqrt(Math.pow(16, 2) + Math.pow(9, 2));
        double fovPerPixel = 68.5 / diagonal;
 
        AprilTagPoseEstimator.Config tempConfig = new AprilTagPoseEstimator.Config(0.1651, (width / 2) / Math.tan((fovPerPixel * 16 / 2) * Math.PI / 180), (height / 2) / Math.tan((fovPerPixel * 9 / 2) * Math.PI / 180), width / 2, height / 2);
        onlyEstimator = new AprilTagPoseEstimator(tempConfig);

        cameraPoseLock = new AtomicBoolean(false);
        updatedPose = false;
    }

    public void update() {
        frontSink.grabFrame(frontImage, 1000000);
        double imageReadTime = Timer.getFPGATimestamp();
        Imgproc.cvtColor(frontImage, grayImage, Imgproc.COLOR_RGB2GRAY);

        /*for(int r = 0; r < frontImage.rows(); r++) {
            for(int c = 0; c < frontImage.cols(); c++) {
                double[] values = frontImage.get(r, c);
                grayImage.put(r, c, 0.299 * values[2] + 0.587 * values[1] + 0.114 * values[0]);
            }
        }*/

        AprilTagDetection[] frontTags = onlyDetector.detect(grayImage);

        Transform3d[] tagEstimates = new Transform3d[frontTags.length];
        Pose3d[] cameraEstimates = new Pose3d[tagEstimates.length];
        int[] tagIDs = new int[tagEstimates.length];
        int[] similar = new int[tagEstimates.length];
        int maxSimilarIndex = 0;

        for(int i = 0; i < frontTags.length; i++) {
            tagEstimates[i] = onlyEstimator.estimate(frontTags[i]);
            //tagEstimates[i] = new Transform3d(tagEstimates[i].getX(), -tagEstimates[i].getZ() - 0.12, tagEstimates[i].getY(), new Rotation3d(tagEstimates[i].getX(), tagEstimates[i].getZ(), tagEstimates[i].getY()));
            tagEstimates[i] = new Transform3d(-tagEstimates[i].getX(), tagEstimates[i].getZ(), -tagEstimates[i].getY(), new Rotation3d(-tagEstimates[i].getRotation().getX() * Math.PI / 180, tagEstimates[i].getRotation().getZ() * Math.PI / 180, -tagEstimates[i].getRotation().getY() * Math.PI / 180));

            tagIDs[i] = frontTags[i].getId();

            /*
            10
            -0.008506944128325297, -0.11274009828347498, 1.2966558338128389
            316.7591131907935, 128.31577303891754
             */
            System.out.println();
            System.out.println(tagIDs[i]);
            printTransform3d(tagEstimates[i]);
            System.out.println(frontTags[i].getCenterX() + ", " + frontTags[i].getCenterY());
        }

        for(int i = 0; i < cameraEstimates.length; i++) {
            double radius = Math.sqrt(Math.pow(tagEstimates[i].getY(), 2) + Math.pow(tagEstimates[i].getZ(), 2));
            double angle = tagEstimates[i].getRotation().getX() + cameraAngle;
            tagEstimates[i] = new Transform3d(tagEstimates[i].getX(), Math.cos(angle) * radius, -Math.sin(angle) * radius, new Rotation3d(angle, tagEstimates[i].getRotation().getY(), tagEstimates[i].getRotation().getZ()));
            printTransform3d(tagEstimates[i]);
            tagEstimates[i] = new Transform3d(tagEstimates[i].getX() + 0.27, tagEstimates[i].getY() + 0.33, tagEstimates[i].getZ(), tagEstimates[i].getRotation());
            printTransform3d(tagEstimates[i]);
            radius = Math.sqrt(Math.pow(tagEstimates[i].getX(), 2) + Math.pow(tagEstimates[i].getY(), 2));
            angle = tagWorldTransforms[tagIDs[i] - 1].getRotation().getZ() + tagEstimates[i].getRotation().getZ();
            tagEstimates[i] = new Transform3d(-Math.sin(angle) * radius, Math.cos(angle) * radius, tagEstimates[i].getZ(), new Rotation3d(tagEstimates[i].getRotation().getX(), tagEstimates[i].getRotation().getY(), angle));
            printTransform3d(tagEstimates[i]);
            cameraEstimates[i] = new Pose3d(tagWorldTransforms[tagIDs[i] - 1].getX() + tagEstimates[i].getX(), 
                tagWorldTransforms[tagIDs[i] - 1].getY() + tagEstimates[i].getY(), 
                tagWorldTransforms[tagIDs[i] - 1].getZ() + tagEstimates[i].getZ(), 
                new Rotation3d(tagWorldTransforms[tagIDs[i] - 1].getRotation().getX() + tagEstimates[i].getRotation().getX(), 
                    tagWorldTransforms[tagIDs[i] - 1].getRotation().getY() + tagEstimates[i].getRotation().getY(), 
                    tagWorldTransforms[tagIDs[i] - 1].getRotation().getZ() + tagEstimates[i].getRotation().getZ()));
            printPose3d(cameraEstimates[i]);
            /*cameraEstimates[i] = tagWorldTransforms[tagIDs[i] - 1].plus(tagEstimates[i].inverse());
            printPose3d(cameraEstimates[i]);
            cameraEstimates[i] = new Pose3d(cameraEstimates[i].getX(), cameraEstimates[i].getY(), cameraEstimates[i].getZ(), cameraEstimates[i].getRotation());
            printPose3d(cameraEstimates[i]);
            cameraEstimates[i] = cameraEstimates[i].plus(frontCameraTransform3d);
            printPose3d(cameraEstimates[i]);*/
        }

        for(int i = 0; i < cameraEstimates.length; i++) {
            for(int j = i + 1; j < cameraEstimates.length; j++) {
                if(j != i) {
                    if(distance(cameraEstimates[i], cameraEstimates[j]) < 0.25) {
                        cameraEstimates[i] = meanPose3d(cameraEstimates[i], cameraEstimates[j], similar[i]);
                        similar[i]++;

                        if(similar[i] > similar[maxSimilarIndex]) {
                            maxSimilarIndex = i;
                        }
                    }
                }
            }
        }

        while(cameraEstimates[maxSimilarIndex].getRotation().getZ() <= 0) {
            cameraEstimates[maxSimilarIndex] = new Pose3d(cameraEstimates[maxSimilarIndex].getX(), 
                cameraEstimates[maxSimilarIndex].getY(), cameraEstimates[maxSimilarIndex].getZ(), 
                new Rotation3d(cameraEstimates[maxSimilarIndex].getRotation().getX(), 
                    cameraEstimates[maxSimilarIndex].getRotation().getY(), 
                    cameraEstimates[maxSimilarIndex].getRotation().getZ() + 360));
        }

        while(cameraEstimates[maxSimilarIndex].getRotation().getZ() > 360) {
            cameraEstimates[maxSimilarIndex] = new Pose3d(cameraEstimates[maxSimilarIndex].getX(), 
                cameraEstimates[maxSimilarIndex].getY(), cameraEstimates[maxSimilarIndex].getZ(), 
                new Rotation3d(cameraEstimates[maxSimilarIndex].getRotation().getX(), 
                    cameraEstimates[maxSimilarIndex].getRotation().getY(), 
                    cameraEstimates[maxSimilarIndex].getRotation().getZ() - 360));
        }

        if(tagEstimates.length != 0) {
            if(cameraPoseLock.get()) {
                try {
                    cameraPoseLock.wait(25);
                } catch(InterruptedException e) {
                    e.printStackTrace();
                }
            }
            
            cameraPoseLock.set(true);
            updatedX = cameraEstimates[maxSimilarIndex].getX();
            updatedY = cameraEstimates[maxSimilarIndex].getY();
            updatedHeading = cameraEstimates[maxSimilarIndex].getRotation().getZ() * 180 / Math.PI;
            updateTime = imageReadTime;
            updatedPose = true;
            cameraPoseLock.set(false);
        }
    }

    public double distance(Pose3d first, Pose3d second) {
        return Math.sqrt(Math.pow(first.getX() - second.getX(), 2) + Math.pow(first.getY() - second.getY(), 2) + Math.pow(first.getZ() - second.getZ(), 2));
    }

    public void printTransform3d(Transform3d rotation) {
        System.out.println(rotation.getX() + ", " + rotation.getY() + ", " + rotation.getZ() + ",\n" + rotation.getRotation().getX() * 180 / Math.PI + ", " + rotation.getRotation().getY() * 180 / Math.PI + ", " +  rotation.getRotation().getZ() * 180 / Math.PI);
    }

    public void printPose3d(Pose3d pose) {
        System.out.println(pose.getX() + ", " + pose.getY() + ", " + pose.getZ() + ",\n" + pose.getRotation().getX() * 180 / Math.PI + ", " + pose.getRotation().getY() * 180 / Math.PI + ", " +  pose.getRotation().getZ() * 180 / Math.PI);
    }

    public Pose3d meanPose3d(Pose3d first, Pose3d second, int similar) {
        return new Pose3d((first.getX() * similar + second.getX()) / (similar + 1), 
            (first.getY() * similar + second.getY()) / (similar + 1), 
            (first.getZ() * similar + second.getZ()) / (similar + 1), 
            new Rotation3d((first.getRotation().getX() * similar + second.getRotation().getX()) / (similar + 1), 
                (first.getRotation().getY() * similar + second.getRotation().getY()) / (similar + 1), 
                (first.getRotation().getZ() * similar + second.getRotation().getZ()) / (similar + 1)));
    }
}
