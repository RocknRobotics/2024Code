package frc.robot;

import java.util.concurrent.atomic.AtomicBoolean;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimate;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    public Transform3d frontCameraTransform3d = new Transform3d(0, 0, 0, new Rotation3d(45 * Math.PI / 180, 0, 180 * Math.PI / 180));

    public AtomicBoolean cameraPoseLock;
    public boolean updatedPose;
    public double updatedX;
    public double updatedY;
    public double updatedHeading;

    public CameraMaster() {
        int width = 640;
        int height = 360;
        frontCamera = new UsbCamera("Front Camera", 0);
        frontCamera.setVideoMode(PixelFormat.kMJPEG, width, height, 1);
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

        for(int r = 0; r < frontImage.rows(); r++) {
            for(int c = 0; c < frontImage.cols(); c++) {
                double[] values = frontImage.get(r, c);
                grayImage.put(r, c, 0.299 * values[2] + 0.587 * values[1] + 0.114 * values[0]);
            }
        }

        AprilTagDetection[] frontTags = onlyDetector.detect(grayImage);

        Transform3d[] tagEstimates = new Transform3d[frontTags.length];
        Pose3d[] cameraEstimates = new Pose3d[tagEstimates.length];
        int[] tagIDs = new int[tagEstimates.length];
        int[] similar = new int[tagEstimates.length];
        int maxSimilarIndex = -1;

        for(int i = 0; i < frontTags.length; i++) {
            tagEstimates[i] = onlyEstimator.estimate(frontTags[i]);
            tagIDs[i] = frontTags[i].getId();

            /*
            10
            -0.008506944128325297, -0.11274009828347498, 1.2966558338128389
            316.7591131907935, 128.31577303891754
             */
            System.out.println(tagIDs[i]);
            System.out.println(tagEstimates[i].getX() + ", " + tagEstimates[i].getY() + ", " + tagEstimates[i].getZ());
            System.out.println(frontTags[i].getCenterX() + ", " + frontTags[i].getCenterY());
        }

        for(int i = 0; i < cameraEstimates.length; i++) {
            cameraEstimates[i] = (tagWorldTransforms[i].transformBy(tagEstimates[i])).transformBy(frontCameraTransform3d);
        }

        for(int i = 0; i < cameraEstimates.length; i++) {
            for(int j = i + 1; j < cameraEstimates.length; j++) {
                if(j != i) {
                    if(distance(cameraEstimates[i], cameraEstimates[j]) < 0.25) {
                        similar[i]++;

                        if(maxSimilarIndex == -1 || similar[i] > similar[maxSimilarIndex]) {
                            maxSimilarIndex = i;
                        }
                    }
                }
            }
        }

        if(maxSimilarIndex != -1) {
            cameraPoseLock.set(true);
            updatedX = cameraEstimates[maxSimilarIndex].getX();
            updatedY = cameraEstimates[maxSimilarIndex].getY();
            updatedHeading = cameraEstimates[maxSimilarIndex].getRotation().getZ();
            updatedPose = true;
            cameraPoseLock.set(false);
        }
    }

    public double distance(Pose3d first, Pose3d second) {
        return Math.sqrt(Math.pow(first.getX() - second.getX(), 2) + Math.pow(first.getY() - second.getY(), 2) + Math.pow(first.getZ() - second.getZ(), 2));
    }
}
