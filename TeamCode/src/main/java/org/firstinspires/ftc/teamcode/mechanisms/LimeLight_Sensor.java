package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.CompetitionTeleOpBlue.BLUE_TAG_IDS;
import static org.firstinspires.ftc.teamcode.CompetitionTeleOpBlue.RED_TAG_IDS;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class LimeLight_Sensor {
    public LimeLight_Sensor(HardwareMap hardwareMap, Telemetry telemetry, double ch, double th, double ma, String s) {
    }

    public void initLimelight(Limelight3A limelight) {
        this.limelight = limelight;
        this.limelightEnabled = (limelight != null);
    }

    public void initAprilTagProcessor(VisionPortal portal, AprilTagProcessor aprilTag) {
        this.visionPortal = portal;
        this.aprilTag = aprilTag;
        this.visionEnabled = (aprilTag != null);
    }
    public static double DISTANCE_CORRECTION = 0.95;

    // === DRIVE ===
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private DcMotor intake;

    // === SHOOTER (CW + CCW on same axle) ===
    private DcMotorEx shooterOne, shootertwo;

    // === HOOD (2 servos opposite) ===
    private Servo hoodOne, hood;

    // === SPINDEXER + KICKER ===
    private Servo spindexer, servoArm;

    // === SENSORS ===
    private ColorSensor colorSenor;
    private Servo led;

    // === VISION ===
    private Limelight3A limelight;
    private boolean limelightEnabled = false;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private boolean visionEnabled = false;
    private boolean isTagIdInAlliance(int tagId) {
        for (int id : BLUE_TAG_IDS) { if (id == tagId) return true; }
        return false;
    }
    public boolean isTagIdGoal(int tagId) {
        for (int id : BLUE_TAG_IDS) { if (id == tagId) return true; }
        for (int id : RED_TAG_IDS) { if (id == tagId) return true; }
        return false;
    }
    /** Has any goal post in view (blue or red) - for Y-button. */
    public boolean hasAnyGoalTarget() {
        if (limelightEnabled && limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null) {
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        if (isTagIdGoal(f.getFiducialId())) return true;
                    }
                }
                if (result.getTa() > 0) return true;
            }
        }
        if (visionEnabled && aprilTag != null) {
            List<AprilTagDetection> det = aprilTag.getDetections();
            for (AprilTagDetection d : det) {
                if (isTagIdGoal(d.id)) return true;
            }
        }
        return false;
    }
    public double getClosestGoalDistanceInches() {
        if (limelightEnabled && limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        if (isTagIdGoal(f.getFiducialId())) {
                            double ta = result.getTa();
                            if (ta > 0) {
                                double d = 72 - (ta / 100.0) * 48;
                                return Math.max(24, Math.min(72, d));
                            }
                            return 48;
                        }
                    }
                    return 48;
                }
                if (result.getTa() > 0) {
                    double d = 72 - (result.getTa() / 100.0) * 48;
                    return Math.max(24, Math.min(72, d));
                }
            }
        }
        if (visionEnabled && aprilTag != null) {
            List<AprilTagDetection> det = aprilTag.getDetections();
            for (AprilTagDetection d : det) {
                if (isTagIdGoal(d.id)) return d.ftcPose.y;
            }
        }
        return 48;
    }
    public double getVisionDistanceInches() {
        double raw = getVisionDistanceInchesRaw();
        return raw * DISTANCE_CORRECTION;
    }
    public double getVisionDistanceInchesRaw() {
        if (limelightEnabled && limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        if (isTagIdInAlliance(f.getFiducialId())) {
                            double ta = result.getTa();
                            if (ta > 0) {
                                double distFromTa = 72 - (ta / 100.0) * 48;
                                return Math.max(24, Math.min(72, distFromTa));
                            }
                            return 48;
                        }
                    }
                    return 48;
                }
                if (result.getTa() > 0) {
                    double distFromTa = 72 - (result.getTa() / 100.0) * 48;
                    return Math.max(24, Math.min(72, distFromTa));
                }
            }
        }
        if (visionEnabled && aprilTag != null) {
            List<AprilTagDetection> det = aprilTag.getDetections();
            for (AprilTagDetection d : det) {
                if (isTagIdInAlliance(d.id)) return d.ftcPose.y;
            }
        }
        return 48;
    }
}
