package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * LIMELIGHT MASON - Refactored vision class.
 * Manages Limelight 3A and AprilTag detections for the Blue Alliance (Mason
 * variant).
 */
public class LimeLightMason {
    public static double DISTANCE_CORRECTION = 0.95;

    // === VISION ===
    private Limelight3A limelight;
    private boolean limelightEnabled = false;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private boolean visionEnabled = false;

    public void initLimelight(Limelight3A limelight) {
        this.limelight = limelight;
        this.limelightEnabled = (limelight != null);
    }

    public void initAprilTagProcessor(VisionPortal portal, AprilTagProcessor aprilTag) {
        this.visionPortal = portal;
        this.aprilTag = aprilTag;
        this.visionEnabled = (aprilTag != null);
    }

    public LLResult getLatestResult() {
        if (this.limelight != null) {
            return this.limelight.getLatestResult();
        }
        return null;
    }

    public boolean isTagIdInAlliance(int tagId) {
        for (int id : CompetitionTeleOpBlueMason.BLUE_TAG_IDS) {
            if (id == tagId)
                return true;
        }
        return false;
    }

    public boolean isTagIdGoal(int tagId) {
        for (int id : CompetitionTeleOpBlueMason.BLUE_TAG_IDS) {
            if (id == tagId)
                return true;
        }
        for (int id : CompetitionTeleOpBlueMason.RED_TAG_IDS) {
            if (id == tagId)
                return true;
        }
        return false;
    }

    /** Has any goal post in view (blue or red). */
    public boolean hasAnyGoalTarget() {
        if (limelightEnabled && limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null) {
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        if (isTagIdGoal(f.getFiducialId()))
                            return true;
                    }
                }
                if (result.getTa() > 0)
                    return true;
            }
        }
        if (visionEnabled && aprilTag != null) {
            List<AprilTagDetection> det = aprilTag.getDetections();
            for (AprilTagDetection d : det) {
                if (isTagIdGoal(d.id))
                    return true;
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
                if (isTagIdGoal(d.id))
                    return d.ftcPose.y;
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
                if (isTagIdInAlliance(d.id))
                    return d.ftcPose.y;
            }
        }
        return 48;
    }

    public double getVisionTxDegrees() {
        if (limelightEnabled && limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        if (isTagIdInAlliance(f.getFiducialId()))
                            return result.getTx();
                    }
                }
                return result.getTx();
            }
        }
        if (visionEnabled && aprilTag != null) {
            List<AprilTagDetection> det = aprilTag.getDetections();
            for (AprilTagDetection d : det) {
                if (isTagIdInAlliance(d.id))
                    return d.ftcPose.bearing;
            }
        }
        return 999;
    }

    public boolean hasVisionTarget() {
        if (limelightEnabled && limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        if (isTagIdInAlliance(f.getFiducialId()))
                            return true;
                    }
                    return false;
                }
                if (result.getTa() > 0)
                    return true;
            }
        }
        if (visionEnabled && aprilTag != null) {
            List<AprilTagDetection> det = aprilTag.getDetections();
            for (AprilTagDetection d : det) {
                if (isTagIdInAlliance(d.id))
                    return true;
            }
        }
        return false;
    }

    public int getVisionTagId() {
        if (limelightEnabled && limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null) {
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        if (isTagIdInAlliance(f.getFiducialId()))
                            return f.getFiducialId();
                    }
                }
            }
        }
        if (visionEnabled && aprilTag != null) {
            List<AprilTagDetection> det = aprilTag.getDetections();
            for (AprilTagDetection d : det) {
                if (isTagIdInAlliance(d.id))
                    return d.id;
            }
        }
        return -1;
    }

    public int getClosestGoalTagId() {
        if (limelightEnabled && limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null) {
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        if (isTagIdGoal(f.getFiducialId()))
                            return f.getFiducialId();
                    }
                }
            }
        }
        if (visionEnabled && aprilTag != null) {
            List<AprilTagDetection> det = aprilTag.getDetections();
            for (AprilTagDetection d : det) {
                if (isTagIdGoal(d.id))
                    return d.id;
            }
        }
        return -1;
    }

    public double getClosestGoalTx() {
        if (limelightEnabled && limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        if (isTagIdGoal(f.getFiducialId()))
                            return result.getTx();
                    }
                }
                return result.getTx();
            }
        }
        if (visionEnabled && aprilTag != null) {
            List<AprilTagDetection> det = aprilTag.getDetections();
            for (AprilTagDetection d : det) {
                if (isTagIdGoal(d.id))
                    return d.ftcPose.bearing;
            }
        }
        return 999;
    }
}

