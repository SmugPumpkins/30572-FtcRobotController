package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import static org.firstinspires.ftc.teamcode.utils.Config.*;
import static org.firstinspires.ftc.teamcode.utils.Constants.*;

import java.util.List;
public class AprilTagLocalizationExample {
    private static final boolean USE_WEBCAM = true;
    private final Position cameraPosition = new Position (
            DistanceUnit.MM,
            140,
            170,
            400,
            0
    );
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(
            AngleUnit.DEGREES,
            0,
            -80,
            0,
            0
    );
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private List<AprilTagDetection> currentDetections;
    private final int BLUE_GOAL_TAG_ID = BLUE_ALLIANCE; // 20
    private final int RED_GOAL_TAG_ID = RED_ALLIANCE; // 24
    private AprilTagLibrary library= AprilTagGameDatabase.getDecodeTagLibrary();
    public void init(HardwareMap hardwareMap){
        aprilTag = buildCamera();
        visionPortal = buildVisionPortal(hardwareMap);
        currentDetections = aprilTag.getDetections();
    }
    public void run(){
        updateDetections();
    }
    public Position getRobotPosition(){
        AprilTagMetadata meta = null;
        if (currentDetections == null || currentDetections.isEmpty()) return null;
        AprilTagDetection tag = null;
        for (AprilTagDetection d : currentDetections) {
            if (d.id == BLUE_GOAL_TAG_ID || d.id == RED_GOAL_TAG_ID) {
                tag = d;
                meta = library.lookupTag(d.id);
                break;
            }
        }
        if (tag == null) return null;
        if (tag.robotPose.getPosition() == null) return null;
        // robot relative to tag
        double robot_x = tag.robotPose.getPosition().x + meta.fieldPosition.get(0);
        double robot_y = tag.robotPose.getPosition().y+ meta.fieldPosition.get(1);
        return new Position(
                DistanceUnit.INCH,
                robot_x,
                robot_y,
                0,
                0
        );
    }
    public YawPitchRollAngles getRobotOrientation(){
        AprilTagMetadata meta = null;
        if (currentDetections == null || currentDetections.isEmpty()) return null;
        AprilTagDetection tag = null;
        for (AprilTagDetection d : currentDetections) {
            if (d.id == BLUE_GOAL_TAG_ID || d.id == RED_GOAL_TAG_ID) {
                tag = d;
                meta = library.lookupTag(d.id);
                break;
            }
        }
        if (tag == null) return null;
        if (tag.robotPose.getOrientation() == null) return null;
        double robot_yaw = tag.robotPose.getOrientation().getYaw(AngleUnit.DEGREES) + quaternionToYawDegrees(meta.fieldOrientation) + 180;
        robot_yaw %= 360;
        robot_yaw -= 180;

        return new YawPitchRollAngles(
                AngleUnit.DEGREES,
                robot_yaw,
                0,
                0,
                0
        );
    }
    public Pose3D position(){
        if (currentDetections == null || currentDetections.isEmpty()) return null;
        AprilTagDetection tag = null;
        for (AprilTagDetection d : currentDetections) {
            if (d.id == BLUE_GOAL_TAG_ID || d.id == RED_GOAL_TAG_ID) {
                tag = d;
                break;
            }
        }
        if (tag == null) return null;
        return tag.robotPose;
    }
    public double getDistanceToGoal(){
        if (currentDetections == null || currentDetections.isEmpty()) return 0;
        AprilTagDetection tag = null;
        for (AprilTagDetection d : currentDetections) {
            if (d.id == BLUE_GOAL_TAG_ID || d.id == RED_GOAL_TAG_ID) {
                tag = d;
                break;
            }
        }
        if (tag == null) return 0;
        return tag.ftcPose.range;
    }
    public void getTelemetry(Telemetry telemetry){
        Pose3D pos = position();
        telemetry.addLine("ROBOT POSITION - WEBCAM 1");
        if (pos == null) {
            telemetry.addLine("No tags are visible...");
            return;
        };
        telemetry.addData("Absolute X", pos.getPosition().x);
        telemetry.addData("Absolute Y", pos.getPosition().x);
        telemetry.addData("Heading", pos.getOrientation().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Distance to Goal", getDistanceToGoal());
        telemetry.addData("Number of Tags", getNumberOfTags());
    }
    public void setDecimation(int decimation){
        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(decimation);
    }
    public void enableAprilTags(){
        visionPortal.setProcessorEnabled(aprilTag, true);
    }
    public void disableAprilTags(){
        visionPortal.setProcessorEnabled(aprilTag, false);
    }
    public int getNumberOfTags(){
        return currentDetections.size();
    }
private void updateDetections(){
    currentDetections = aprilTag.getDetections();
}
    private AprilTagProcessor buildCamera(){
        AprilTagProcessor builder;
        builder = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();
        return builder;
    }
    private VisionPortal buildVisionPortal(HardwareMap hardwareMap){
        VisionPortal portal;
        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, WEBCAM));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        portal = builder.build();
        return portal;
    }
    private double quaternionToYawDegrees(Quaternion q) {
        double w = q.w;
        double x = q.x;
        double y = q.y;
        double z = q.z;

        double yawRad = Math.atan2(
                2.0 * (w * z + x * y),
                1.0 - 2.0 * (y * y + z * z)
        );

        return Math.toDegrees(yawRad);
    }
}

