package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

/**
 * COMPETITION TELEOP BLUE - Full shooter system. BLUE ALLIANCE ONLY.
 * Load "Competition TeleOp BLUE" for Blue matches. For Red use CompetitionTeleOpRed.
 *
 * VISION: LimeLight 3A primary (tx, ty, ta, fiducial); Webcam AprilTag fallback
 * LED: Color sensor -> ball color (green/purple) displayed on LED
 *
 * HARDWARE:
 *   Control Hub: intake, rightFront, rightBack, shooterOne | hoodOne, spindexer, servoArm, led | colorSenor
 *   Expansion Hub: shooterTwo, leftFront, leftBack | hood
 *
 * RULES:
 *   - Shooter: BOTH motors SAME AXLE, one CW one CCW (or gears damaged)
 *   - Hood: 2 servos opposite directions
 *   - Kicker and spindexer NEVER move together
 *
 * CONTROLS (ONE GAMEPAD - all on G1):
 *   Left Stick:         X=forward/back (L=fwd R=back), Y=strafe (U=right D=left). Front=shooting.
 *   Right Stick X:      Rotate
 *   Left Trigger:       Intake (hold) - full RPM, spindexer cycles 0/0.063/0.580; after release intake runs 3 sec while cycling so balls settle
 *   Left Bumper:        Shooter OFF
 *   Right Bumper/Trigger: SHOOT (hold) - auto if AprilTag; manual: 3s spinup then fire
 *   A:                  Velocity preset (cycle)
 *   B:                  Spindexer step (slot 0->1->2 when idle OR during indexing - all 3 locations)
 *   Y:                  AUTO-DRIVE toward AprilTag (hold = rotate + drive forward)
 *   D-pad Left:         Hood down
 *   D-pad Right:        Hood up
 *   LT + D-pad Up:      Intake REVERSE (release stuck balls)
 *
 * (G2 same as G1 if second gamepad connected)
 */

@TeleOp(name = "Competition TeleOp BLUE", group = "Driver")
public class CompetitionTeleOpBlue extends LinearOpMode {

    // === DRIVE (18" x 18" FTC-legal robot - fast top speed, driver-friendly curve) ===
    public static double MAX_SPEED = 1.0;           // full speed for big robot
    public static double PRECISION_SPEED = 0.5;     // D-pad Down held = slower for alignment/confidence
    public static double DRIVE_CURVE_EXPONENT = 2.0;  // squared curve: small stick = fine control, full stick = full power
    public static double STRAFE_MULTIPLIER = 0.9;
    public static double ROTATE_MULTIPLIER = 1.0;
    public static double DEAD_ZONE = 0.08;          // slightly more responsive
    public static double INTAKE_POWER = 0.8;

    // === SHOOTER - distance-based from FTC positions (anchor table + interpolation) ===
    public static double VELOCITY_MIN = 800;
    public static double VELOCITY_MAX = 2400;
    /** Anchor: corrected distance (in) -> velocity (ticks/s). Calibrated: 68 in -> 1350 RPM. */
    public static double[] SHOOTER_ANCHOR_DIST_IN = { 48, 68, 90 };
    public static double[] SHOOTER_ANCHOR_VELOCITY = { 1100, 1350, 1700 };
    /** Anchor: corrected distance (in) -> hood position [0,1]. Lower hood at close = ty ~0. */
    public static double[] SHOOTER_ANCHOR_HOOD = { 0.55, 0.72, 0.95 };
    /** Backspin adds lift: reduce RPM or lower hood. 1.0 = no change; try 0.95 if shots go high. */
    public static double BACKSPIN_RPM_FACTOR = 1.0;
    /** Backspin: add to hood (negative = lower hood). Try -0.03 if shots go high. */
    public static double BACKSPIN_HOOD_OFFSET = 0.0;
    public static double VELOCITY_DEFAULT = 1350;  // fallback when no vision
    /** Far-side: use max power (RPM). Vision distance >= this = far shot. */
    public static double FAR_DISTANCE_THRESHOLD_IN = 120.0;   // inches
    public static double VELOCITY_FAR_SIDE_MAX = 6000.0;    // RPM for far shots

    // === HOOD (5-turn torque motors, inverted: lower value = higher position) ===
    public static double HOOD_POS_TOP = 0.180;      // rack top (high hood, far shots)
    public static double HOOD_POS_BOTTOM = 0.500;   // rack bottom (low hood, close shots)
    public static double HOOD_DIST_NEAR = 24;
    public static double HOOD_DIST_FAR = 72;

    // === SPINDEXER: goBILDA servo (2.25 turns = 810deg usable range) ===
    // Physical order: 1-0-2 (slot 1 is lowest position)
    // Rotation 1 (rev 0): Shoot_1=0.010, Shoot_0=0.073, Shoot_2=0.137
    // Rotation 2 (rev 1): Shoot_1=0.200, Shoot_0=0.263, Shoot_2=0.329
    // Rotation 3 (rev 2): Shoot_1=0.396 (max before auto-reset)
    public static double SPINDEXER_SHOOT_0 = 0.073;   // Rotation 1, slot 0
    public static double SPINDEXER_SHOOT_1 = 0.010;   // Rotation 1, slot 1 (lowest)
    public static double SPINDEXER_SHOOT_2 = 0.137;   // Rotation 1, slot 2 (highest)
    public static double SPINDEXER_INTAKE_0 = 0.066;  // shoot_0 - 0.007 (estimated)
    public static double SPINDEXER_INTAKE_1 = 0.003;  // shoot_1 - 0.007 (estimated)
    public static double SPINDEXER_INTAKE_2 = 0.130;  // shoot_2 - 0.007 (estimated)
    public static double SPINDEXER_REV_OFFSET = 0.190; // ~0.190 per 360deg revolution (measured)
    public static int    SPINDEXER_MAX_FWD_REV = 2;   // max forward (rev 2: slot 1 at 0.396, near reset)
    public static int    SPINDEXER_MAX_BWD_REV = 0;   // cannot go backward (rev 0 is minimum)
    /**
     * Sensor rotation mapping sign. The 3 color sensors are FIXED on the robot frame;
     * as the turntable rotates, different logical slots pass under different sensors.
     * +1: sensor[s] sees slot (s + activeSlot) % 3   (try this first)
     * -1: sensor[s] sees slot (s - activeSlot + 3) % 3  (if +1 is wrong, flip to -1)
     * TEST: put a ball in ONLY slot 0. Command spindexer to SHOOT_1. Check telemetry.
     *       If Slot 0 shows BALL: sign is correct. If wrong slot shows BALL: flip sign.
     */
    public static int SENSOR_ROTATION_SIGN = 1;
    public static int SPINDEXER_INDEX_MS = 250;           // time at each slot (shoot) - faster cycle
    public static int SPINDEXER_INTAKE_CYCLE_MS = 450;    // ms per step (0->1->2->2->1->0) - slower so balls don't fly off
    public static double INTAKE_COAST_SEC = 3.0;         // after release trigger, intake runs this long while spindexer cycles

    // === KICKER (servoArm: up = ball out, down = ready for next) ===
    public static double KICKER_UP_POS = 0.060;
    public static double KICKER_DOWN_POS = 0.330;
    public static int KICKER_UP_MS = 220;   // long enough for plate to move so ball reliably goes out
    public static int KICKER_DOWN_MS = 50;   // fast return, then spin for next ball

    // === ALIGNMENT ===
    public static double ALIGNMENT_TOLERANCE_DEG = 2.0;
    /** Alignment target: tx = 0 means aligned at goal. Set non-zero only if camera/robot has fixed offset. */
    public static double ALIGNMENT_TX_OFFSET_DEG = 0.0;
    /** If Y/R2 makes tx worse (e.g. ±22 instead of ~0), set to -1 to reverse correction direction. */
    public static double ALIGNMENT_CORRECTION_SIGN = -1.0;
    public static double ALIGNMENT_STRAFE_GAIN = 0.03;  // auto-strafe when aligning (reduce if overshoot on wood)
    public static double ALIGNMENT_ROTATE_GAIN = 0.04;  // auto-rotate toward goal (reduce if overshoot on wood)
    public static double AUTO_DRIVE_ROTATE_GAIN = 0.02; // auto-rotate toward target (Y button)
    public static double AUTO_DRIVE_FORWARD_GAIN = 0.015; // auto-drive forward based on distance
    /** Slight press LT = drive with intake as front (180°); slight press RT/RB = drive with shooter as front. */
    public static double INTAKE_ORIENTATION_TRIGGER_THRESHOLD = 0.2;

    // === DEFENSIVE DRIVE (Pinpoint position hold during shooting - resist being pushed) ===
    public static boolean DEFENSIVE_LOCK_ENABLED = true;
    public static double DEFENSIVE_LOCK_KP_X = 0.03;      // position gain (inches error -> forward/strafe)
    public static double DEFENSIVE_LOCK_KP_Y = 0.03;
    public static double DEFENSIVE_LOCK_KP_HEADING = 0.02;  // heading gain (degrees error -> rotate)
    public static double DEFENSIVE_LOCK_DEADBAND_IN = 0.5;   // ignore small position errors
    public static double DEFENSIVE_LOCK_DEADBAND_DEG = 2.0;  // ignore small heading errors
    public static double DEFENSIVE_LOCK_MAX_CORRECTION = 0.4;  // cap corrective power per axis

    // === MANUAL SHOOT (override when no AprilTag) - FASTEST THROUGHPUT ===
    public static double MANUAL_SHOOT_POWER = 0.8;  // 80% power when manual override
    public static double MANUAL_SHOOT_SPINUP_SEC = 1.0;  // 1 sec spinup for first ball only
    public static int MANUAL_INDEX_MS = 250;         // time at each slot - faster cycle
    public static int MANUAL_KICKER_UP_MS = 200;     // long enough for plate to move so ball reliably goes out
    public static int MANUAL_KICKER_DOWN_MS = 50;    // fast return, then spin for next ball

    // === LED (your unit: A=off 0.0, B=red 0.35, X=green 0.500, Y=white 0.722; purple 0.666) ===
    // goBILDA: below 1100µs light is OFF. Use 0.35 so B always turns light ON (red/orange).
    public static double LED_POS_OFF = 0.0;           // A = off (light off)
    public static double LED_POS_RED = 0.35;         // B = red = no ball (0.35 = well above off threshold)
    public static double LED_POS_YELLOW = 0.388;      // AprilTag detected (blink)
    public static double LED_POS_GREEN_BALL = 0.500;  // X = green ball
    public static double LED_POS_PURPLE_BALL = 0.666; // Indigo (purple ball; Y is white on your unit)
    public static double LED_POS_WHITE = 0.722;        // Y = white
    public static int LED_BLINK_MS = 250;            // AprilTag blink interval
    public static int BALL_MIN_BRIGHTNESS = 50;     // below this = no ball (e.g. R22 G40 B35 total 97)
    public static int BALL_GREEN_DOMINANCE = 5;      // green: g > r+5 and g > b+5 (e.g. R29 G68 B58)
    public static int BALL_PURPLE_MIN = 30;          // purple: r,b both > 30 and r+b > g+5 (e.g. R38 G58 B66)

    // === GOAL POST TAGS (FTC Game Manual 9.10 - DECODE: GOAL only, not OBELISK) ===
    public static int[] BLUE_TAG_IDS = { 20 };   // Blue Alliance GOAL - per manual
    public static int[] RED_TAG_IDS = { 24 };    // Red Alliance GOAL - per manual (for telemetry only)
    /** Limelight = tag plane; goal opening BEHIND tag (FTC). Corrected = raw × this. */
    public static double DISTANCE_CORRECTION = 0.95;
    /**
     * Ty-based distance estimation: distance = heightDiff / tan(cameraTilt + ty).
     * MUCH more accurate than ta-based. CALIBRATE camera height and tilt for your robot!
     * TAG_HEIGHT = height of AprilTag center from floor (FTC Into The Deep goal = 37").
     * CAMERA_HEIGHT = your Limelight lens height from floor.
     * CAMERA_TILT = camera's upward tilt angle from horizontal (degrees).
     * If ty reads ~0 when looking straight at a tag at the same height → tilt ≈ 0.
     */
    public static double VISION_TAG_HEIGHT_IN = 37.0;      // AprilTag center height (inches)
    public static double VISION_CAMERA_HEIGHT_IN = 12.0;   // Camera lens height (inches) - CALIBRATE!
    public static double VISION_CAMERA_TILT_DEG = 20.0;    // Camera upward tilt (degrees) - CALIBRATE!

    // === DRIVE ===
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private DcMotor intake;
    private IMU imu;

    // === SHOOTER (CW + CCW on same axle) ===
    private DcMotorEx shooterOne, shootertwo;

    // === HOOD (2 servos opposite) ===
    private Servo hoodOne, hood;

    // === SPINDEXER + KICKER ===
    private Servo spindexer, servoArm;

    // === SENSORS ===
    private ColorSensor colorSenor;   // color sensor 1 - fixed position, sees slot 1 when slot 0 is active
    private ColorSensor cs2;          // color sensor 2 - fixed position, sees slot 2 when slot 0 is active
    private ColorSensor cs3;          // color sensor 3 - fixed position, sees slot 0 when slot 0 is active
    private Servo led;

    // === VISION ===
    private Limelight3A limelight;
    private boolean limelightEnabled = false;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private boolean visionEnabled = false;

    // === PINPOINT ODOMETRY (disabled - add GoBildaPinpointDriver.java + uncomment init to enable) ===
    private GoBildaPinpointDriver pinpoint = null;
    private boolean pinpointEnabled = false;

    // === STATE ===
    private double targetVelocity = VELOCITY_DEFAULT;
    private int velocityPreset = 1;
    private boolean intakeRunning = false;
    /** B = toggle: when true, intake + spindexer cycle run until B pressed again. */
    private boolean intakeCycleToggle = false;
    private double hoodPosition = 0.5;
    private boolean flywheelSpinning = false;
    private boolean prevG1LB, prevG2RB, prevG2LB, prevG2A, prevB;
    private int manualSpindexerSlot = 0;
    private boolean intakeCoastActive = false;  // after release trigger, intake runs 3 sec while cycling
    private ElapsedTime intakeCoastTimer = new ElapsedTime();
    private int intakeCycleStep = 0;         // current slot index (0-2) during intake cycling
    private int spindexerRevolution = 0;     // current revolution (0 = start, grows forward/backward)
    private boolean spindexerForward = true;  // current cycling direction (true = increasing position)
    private ElapsedTime intakeCycleTimer = new ElapsedTime();
    private ElapsedTime ledBlinkTimer = new ElapsedTime();
    /** true = stick forward moves toward intake (180° from shooter); false = shooter is front. */
    private boolean driveOrientationIntakeFront = false;

    // One-button shoot state machine
    private enum ShootState { IDLE, ALIGNING, INDEXING, KICKER_UP, KICKER_DOWN, DONE }
    private ShootState shootState = ShootState.IDLE;
    private boolean manualShootMode = false;  // true = no vision, 80% power, skip alignment
    private int currentSpindexerSlot = 0;
    private ElapsedTime stateTimer = new ElapsedTime();
    private boolean prevShootBumper = false;

    // Defensive drive (position hold during shooting)
    private boolean defensiveLockActive = false;
    private double lockTargetX = 0, lockTargetY = 0, lockTargetHeading = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        initHardware();
        stopAll();
        telemetry.addData("Status", "Ready! Right Bumper = One-Button Shoot");
        telemetry.addData("Code Version", 8);
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (pinpoint != null) pinpoint.update();
            updateDrive();
            updateIntake();
            updateIntakeSpindexer();
            updateShootSequence();
            updateHood();
            updateLedFromColor();
            updateTelemetry();
        }
        stopAll();
        if (limelight != null && limelightEnabled) limelight.stop();
        if (visionPortal != null) visionPortal.close();
    }

    /** True if tag ID is in Blue alliance list. */
    private boolean isTagIdInAlliance(int tagId) {
        for (int id : BLUE_TAG_IDS) { if (id == tagId) return true; }
        return false;
    }

    /** True if tag ID is any goal post (blue or red) - for Y-button align to closest. */
    private boolean isTagIdGoal(int tagId) {
        for (int id : BLUE_TAG_IDS) { if (id == tagId) return true; }
        for (int id : RED_TAG_IDS) { if (id == tagId) return true; }
        return false;
    }

    /** Has any goal post in view (blue or red) - for Y-button. */
    private boolean hasAnyGoalTarget() {
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

    /** Tx (degrees) to closest goal in view (blue or red). 999 if none. */
    private double getClosestGoalTx() {
        if (limelightEnabled && limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        if (isTagIdGoal(f.getFiducialId())) return result.getTx();
                    }
                    return 999;
                }
                return result.getTx();
            }
        }
        if (visionEnabled && aprilTag != null) {
            List<AprilTagDetection> det = aprilTag.getDetections();
            for (AprilTagDetection d : det) {
                if (isTagIdGoal(d.id)) return d.ftcPose.bearing;
            }
        }
        return 999;
    }

    /** Distance (inches) to closest goal in view. 48 if none. */
    private double getClosestGoalDistanceInches() {
        if (limelightEnabled && limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        if (isTagIdGoal(f.getFiducialId())) {
                            // PRIMARY: ty-based distance
                            double ty = result.getTy();
                            double tyDist = distanceFromTy(ty);
                            if (tyDist > 0) return tyDist;
                            // FALLBACK: ta-based
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
                double ty = result.getTy();
                double tyDist = distanceFromTy(ty);
                if (tyDist > 0) return tyDist;
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

    /** Tag ID of closest goal in view, or -1. */
    private int getClosestGoalTagId() {
        if (limelightEnabled && limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null) {
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        if (isTagIdGoal(f.getFiducialId())) return f.getFiducialId();
                    }
                }
            }
        }
        if (visionEnabled && aprilTag != null) {
            List<AprilTagDetection> det = aprilTag.getDetections();
            for (AprilTagDetection d : det) {
                if (isTagIdGoal(d.id)) return d.id;
            }
        }
        return -1;
    }

    /** Returns distance to goal in inches (corrected for goal plane behind tag). LimeLight primary. Alliance-filtered. */
    private double getVisionDistanceInches() {
        double raw = getVisionDistanceInchesRaw();
        return raw * DISTANCE_CORRECTION;
    }

    /**
     * Compute distance from Limelight ty (vertical angle) using trig:
     *   distance = (TAG_HEIGHT - CAMERA_HEIGHT) / tan(CAMERA_TILT + ty)
     * Returns -1 if ty is unusable (angle too shallow or negative).
     */
    private double distanceFromTy(double tyDeg) {
        double heightDiff = VISION_TAG_HEIGHT_IN - VISION_CAMERA_HEIGHT_IN;
        double totalAngleDeg = VISION_CAMERA_TILT_DEG + tyDeg;
        // Guard: angle must be positive and reasonable (0.5°–60°)
        if (totalAngleDeg < 0.5 || totalAngleDeg > 60) return -1;
        double dist = heightDiff / Math.tan(Math.toRadians(totalAngleDeg));
        return Math.max(24, Math.min(120, dist));
    }

    private double getVisionDistanceInchesRaw() {
        if (limelightEnabled && limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        if (isTagIdInAlliance(f.getFiducialId())) {
                            // PRIMARY: ty-based distance (much more reliable than ta)
                            double ty = result.getTy();
                            double tyDist = distanceFromTy(ty);
                            if (tyDist > 0) return tyDist;
                            // FALLBACK: ta-based (only if ty is unusable)
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
                // No fiducials matched but result valid — try global ty
                double ty = result.getTy();
                double tyDist = distanceFromTy(ty);
                if (tyDist > 0) return tyDist;
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

    /** Returns horizontal angle to goal in degrees (tx / bearing). LimeLight primary. Alliance-filtered. */
    private double getVisionTxDegrees() {
        if (limelightEnabled && limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        if (isTagIdInAlliance(f.getFiducialId())) return result.getTx();
                    }
                    return 999;
                }
                return result.getTx();
            }
        }
        if (visionEnabled && aprilTag != null) {
            List<AprilTagDetection> det = aprilTag.getDetections();
            for (AprilTagDetection d : det) {
                if (isTagIdInAlliance(d.id)) return d.ftcPose.bearing;
            }
        }
        return 999;
    }

    /** Returns true if vision has a valid target for the selected alliance. */
    private boolean hasVisionTarget() {
        if (limelightEnabled && limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        if (isTagIdInAlliance(f.getFiducialId())) return true;
                    }
                    return false;
                }
                if (result.getTa() > 0) return true;
            }
        }
        if (visionEnabled && aprilTag != null) {
            List<AprilTagDetection> det = aprilTag.getDetections();
            for (AprilTagDetection d : det) {
                if (isTagIdInAlliance(d.id)) return true;
            }
        }
        return false;
    }

    /** Returns the tag ID of the first vision target matching the selected alliance, or -1. */
    private int getVisionTagId() {
        if (limelightEnabled && limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null) {
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        if (isTagIdInAlliance(f.getFiducialId())) return f.getFiducialId();
                    }
                }
            }
        }
        if (visionEnabled && aprilTag != null) {
            List<AprilTagDetection> det = aprilTag.getDetections();
            for (AprilTagDetection d : det) {
                if (isTagIdInAlliance(d.id)) return d.id;
            }
        }
        return -1;
    }

    private void initHardware() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU for field-relative driving (from MecanumDriveTrain)
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterOne = hardwareMap.get(DcMotorEx.class, "shooterOne");
        shootertwo = hardwareMap.get(DcMotorEx.class, "shooterTwo");
        shooterOne.setDirection(DcMotor.Direction.REVERSE);
        shootertwo.setDirection(DcMotor.Direction.FORWARD);
        shooterOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootertwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidf = new PIDFCoefficients(300, 0, 0, 10);
        shooterOne.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shootertwo.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        hoodOne = hardwareMap.get(Servo.class, "hoodOne");
        hood = hardwareMap.get(Servo.class, "hood");

        spindexer = hardwareMap.get(Servo.class, "spindexer");
        servoArm = hardwareMap.get(Servo.class, "servoArm");

        // goBILDA 5-turn (1800 deg) servo: full range 500-2500 usec
        if (spindexer instanceof ServoImplEx) {
            ((ServoImplEx) spindexer).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
        if (servoArm instanceof ServoImplEx) {
            ((ServoImplEx) servoArm).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
        // goBILDA 5-turn torque motors: full range 500-2500 usec (position limits: 0.180 top, 0.500 bottom)
        if (hoodOne instanceof ServoImplEx) {
            ((ServoImplEx) hoodOne).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
        if (hood != null && hood instanceof ServoImplEx) {
            ((ServoImplEx) hood).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }

        try { colorSenor = hardwareMap.get(ColorSensor.class, "colorSenor"); } catch (Exception e) { colorSenor = null; }
        if (colorSenor == null) try { colorSenor = hardwareMap.get(ColorSensor.class, "colorSensor"); } catch (Exception e2) {}
        if (colorSenor != null) {
            try { colorSenor.getClass().getMethod("enableLed", boolean.class).invoke(colorSenor, true); } catch (Exception e) {}
        }
        // cs2 & cs3: extra color sensors for smart spindexer (ball presence per slot)
        // Sensors are FIXED on the frame — they do NOT rotate with the turntable.
        try { cs2 = hardwareMap.get(ColorSensor.class, "cs2"); } catch (Exception e) { cs2 = null; }
        if (cs2 != null) {
            try { cs2.getClass().getMethod("enableLed", boolean.class).invoke(cs2, true); } catch (Exception e) {}
        }
        try { cs3 = hardwareMap.get(ColorSensor.class, "cs3"); } catch (Exception e) { cs3 = null; }
        if (cs3 != null) {
            try { cs3.getClass().getMethod("enableLed", boolean.class).invoke(cs3, true); } catch (Exception e) {}
        }
        try { led = hardwareMap.get(Servo.class, "led"); } catch (Exception e) { led = null; }
        if (led != null && led instanceof ServoImplEx) ((ServoImplEx) led).setPwmRange(new PwmControl.PwmRange(500, 2500));

        // Pinpoint odometry: orientation + pose for end-game auto-park (shortest path to end game area)
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
            pinpoint.setOffsets(-2.5, 1, DistanceUnit.INCH);
            pinpoint.resetPosAndIMU();
            pinpointEnabled = true;
        } catch (Exception e) { pinpoint = null; pinpointEnabled = false; }

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(0);  // use pipeline 0 - set to AprilTag in web UI
            limelight.start();
            limelightEnabled = true;
        } catch (Exception e) { limelight = null; limelightEnabled = false; }

        try {
            aprilTag = new AprilTagProcessor.Builder().setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f).build();
            visionPortal = new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).addProcessor(aprilTag).build();
            visionEnabled = true;
        } catch (Exception e) { visionEnabled = false; }
    }

    private void updateDrive() {
        // X button = reset IMU yaw (field-relative recalibration)
        if (gamepad1.x || gamepad2.x) {
            imu.resetYaw();
        }

        // Read gamepad inputs (standard FTC mapping - matches MecanumDriveTrain)
        double forward = -gamepad1.left_stick_y;   // left stick Y = forward/back
        double right = gamepad1.left_stick_x;     // left stick X = strafe right/left
        double rotate = -gamepad1.right_stick_x;  // right stick X = rotation (negated)

        // Field-relative driving using IMU (from MecanumDriveTrain.driveFieldRelative)
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);
        theta = AngleUnit.normalizeRadians(theta - robotYaw);
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);
        forward = newForward;
        double strafe = newRight;

        // Y button: whole robot auto-aligns to BLUE goal (Tag 20) - rotate + strafe until tx ~0
        if ((gamepad1.y || gamepad2.y) && hasVisionTarget()) {
            double tx = getVisionTxDegrees();
            double txErr = (Math.abs(tx) < 999) ? (tx - ALIGNMENT_TX_OFFSET_DEG) : 0;
            double dist = getVisionDistanceInches();
            if (Math.abs(txErr) > ALIGNMENT_TOLERANCE_DEG) {
                double corr = ALIGNMENT_CORRECTION_SIGN * txErr;
                rotate = corr * ALIGNMENT_ROTATE_GAIN;
                strafe += corr * ALIGNMENT_STRAFE_GAIN;
            }
            if (dist > 24 && dist < 999) {
                forward = (dist - 24) * AUTO_DRIVE_FORWARD_GAIN;
                if (forward > 0.5) forward = 0.5;
            }
        }

        // Auto-strafe and auto-rotate during ALIGNING (R2 shoot)
        if (shootState == ShootState.ALIGNING && hasVisionTarget()) {
            double tx = getVisionTxDegrees();
            double txErr = (Math.abs(tx) < 999) ? (tx - ALIGNMENT_TX_OFFSET_DEG) : 0;
            if (Math.abs(txErr) > ALIGNMENT_TOLERANCE_DEG) {
                double corr = ALIGNMENT_CORRECTION_SIGN * txErr;
                strafe += corr * ALIGNMENT_STRAFE_GAIN;
                rotate += corr * ALIGNMENT_ROTATE_GAIN;
            }
        }

        // Defensive drive: Pinpoint position hold during shooting - resist being pushed
        boolean shouldLock = DEFENSIVE_LOCK_ENABLED && pinpoint != null && pinpointEnabled
                && shootState != ShootState.IDLE && shootState != ShootState.DONE;
        if (shouldLock) {
            if (!defensiveLockActive) {
                lockTargetX = pinpoint.getPosX(DistanceUnit.INCH);
                lockTargetY = pinpoint.getPosY(DistanceUnit.INCH);
                lockTargetHeading = pinpoint.getHeading(AngleUnit.DEGREES);
                defensiveLockActive = true;
            }
            double currX = pinpoint.getPosX(DistanceUnit.INCH);
            double currY = pinpoint.getPosY(DistanceUnit.INCH);
            double currH = pinpoint.getHeading(AngleUnit.DEGREES);
            double errX = lockTargetX - currX;
            double errY = lockTargetY - currY;
            double errH = wrapDeg(lockTargetHeading - currH);
            if (Math.abs(errX) < DEFENSIVE_LOCK_DEADBAND_IN) errX = 0;
            if (Math.abs(errY) < DEFENSIVE_LOCK_DEADBAND_IN) errY = 0;
            if (Math.abs(errH) < DEFENSIVE_LOCK_DEADBAND_DEG) errH = 0;
            double defTheta = Math.toRadians(currH);
            double forwardErr = errX * Math.cos(defTheta) + errY * Math.sin(defTheta);
            double strafeErr = -errX * Math.sin(defTheta) + errY * Math.cos(defTheta);
            double correctiveForward = Math.max(-DEFENSIVE_LOCK_MAX_CORRECTION, Math.min(DEFENSIVE_LOCK_MAX_CORRECTION, DEFENSIVE_LOCK_KP_X * forwardErr));
            double correctiveStrafe = Math.max(-DEFENSIVE_LOCK_MAX_CORRECTION, Math.min(DEFENSIVE_LOCK_MAX_CORRECTION, DEFENSIVE_LOCK_KP_Y * strafeErr));
            double correctiveRotate = Math.max(-DEFENSIVE_LOCK_MAX_CORRECTION, Math.min(DEFENSIVE_LOCK_MAX_CORRECTION, DEFENSIVE_LOCK_KP_HEADING * errH));
            forward += correctiveForward;
            strafe += correctiveStrafe;
            rotate += correctiveRotate;
        } else {
            defensiveLockActive = false;
        }

        // Dead zone
        if (Math.abs(forward) < DEAD_ZONE) forward = 0;
        if (Math.abs(strafe) < DEAD_ZONE) strafe = 0;
        if (Math.abs(rotate) < DEAD_ZONE) rotate = 0;

        // Input curve: small stick = fine control, full stick = full power (driver confidence)
        if (forward != 0) forward = Math.signum(forward) * Math.pow(Math.abs(forward), DRIVE_CURVE_EXPONENT);
        if (strafe != 0) strafe = Math.signum(strafe) * Math.pow(Math.abs(strafe), DRIVE_CURVE_EXPONENT);
        if (rotate != 0) rotate = Math.signum(rotate) * Math.pow(Math.abs(rotate), DRIVE_CURVE_EXPONENT);
        strafe *= STRAFE_MULTIPLIER;
        rotate *= ROTATE_MULTIPLIER;

        // Mecanum formula (from MecanumDriveTrain.drive)
        double frontLeftPower = forward + rotate + strafe;
        double frontRightPower = forward - rotate - strafe;
        double backRightPower = forward - rotate + strafe;
        double backLeftPower = forward + rotate - strafe;

        // Normalize (from MecanumDriveTrain.drive)
        double maxPower = 1.0;
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        double speedCap = (gamepad1.dpad_down || gamepad2.dpad_down) ? PRECISION_SPEED : MAX_SPEED;
        leftFront.setPower(speedCap * (frontLeftPower / maxPower));
        rightFront.setPower(speedCap * (frontRightPower / maxPower));
        leftBack.setPower(speedCap * (backLeftPower / maxPower));
        rightBack.setPower(speedCap * (backRightPower / maxPower));
    }

    private void updateIntake() {
        // B = toggle indexing: press once = run intake+cycle until press again
        boolean bNow = gamepad1.b || gamepad2.b;
        if (bNow && !prevB) intakeCycleToggle = !intakeCycleToggle;
        prevB = bNow;

        boolean lt = gamepad1.left_trigger > 0.5 || gamepad2.left_trigger > 0.5;
        boolean dpadUp = gamepad1.dpad_up || gamepad2.dpad_up;
        boolean intakeReverse = lt && dpadUp;
        boolean intakeForward = (lt && !dpadUp) || intakeCycleToggle;

        if (intakeForward && !intakeRunning) {
            intakeRunning = true;
            intakeCycleStep = 0;
            intakeCycleTimer.reset();
            // Smart intake: start at first empty slot immediately
            if (hasSmartSensors()) {
                int empty = findNextEmptySlot(0, manualSpindexerSlot);
                if (empty >= 0) intakeCycleStep = empty;
            }
        }
        if (intakeReverse && !intakeRunning) {
            intakeRunning = true;
            intakeCycleStep = 0;
            intakeCycleTimer.reset();
        }
        if (!intakeForward && intakeRunning) {
            intakeRunning = false;
            intakeCoastActive = true;
            intakeCoastTimer.reset();
            intakeCycleStep = 0;
            intakeCycleTimer.reset();
        }
        if (intakeCoastActive && intakeCoastTimer.seconds() >= INTAKE_COAST_SEC) {
            intakeCoastActive = false;
        }
        boolean intakeMotorOn = intakeRunning || intakeCoastActive;
        if (intakeMotorOn) {
            intake.setPower(intakeReverse ? -INTAKE_POWER : INTAKE_POWER);
        } else {
            intake.setPower(0);
        }
    }

    private void updateIntakeSpindexer() {
        if (spindexer == null) return;
        if (shootState != ShootState.IDLE && shootState != ShootState.DONE) return;

        if (intakeRunning || intakeCoastActive) {
            if (hasSmartSensors()) {
                // === SMART INTAKE: only move to EMPTY slots (multi-revolution) ===
                // Sensors are fixed; turntable rotates. At current position, check if
                // current slot has a ball. If yes, advance one slot in the current direction.
                if (intakeCycleTimer.milliseconds() >= SPINDEXER_INTAKE_CYCLE_MS) {
                    intakeCycleTimer.reset();
                    if (isSlotOccupied(intakeCycleStep, intakeCycleStep)) {
                        // Ball loaded → advance one slot in current direction
                        advanceSpindexerSlot();
                    }
                    // If slot still empty: stay and wait for ball to land
                }
                spindexer.setPosition(getSpindexerIntakePos(intakeCycleStep));
            } else {
                // Fallback (no smart sensors): continuous multi-revolution cycling
                // 5-turn servo: 0→1→2→0→1→2→... (forward), then 2→1→0→2→1→0→... (reverse)
                if (intakeCycleTimer.milliseconds() >= SPINDEXER_INTAKE_CYCLE_MS) {
                    intakeCycleTimer.reset();
                    advanceSpindexerSlot();
                }
                spindexer.setPosition(getSpindexerIntakePos(intakeCycleStep));
            }
            return;
        }

        // B is now toggle for indexing (handled in updateIntake). When idle and not cycling, hold current manual slot.
        spindexer.setPosition(getSpindexerShootPos(manualSpindexerSlot));
    }

    private void updateShootSequence() {
        boolean shootBumper = gamepad1.right_bumper || gamepad2.right_bumper
                || gamepad1.right_trigger > 0.5 || gamepad2.right_trigger > 0.5;

        if (shootBumper && !prevShootBumper && shootState == ShootState.IDLE) {
            // Reset revolution to 0 for shooting - ensures positions align with kicker plate
            spindexerRevolution = 0;
            // COMMENTED OUT: Smart sensor ball detection - testing sequential cycling
            // if (hasSmartSensors()) {
            //     // Smart: only start if there's a ball to shoot
            //     int firstOccupied = findNextOccupiedSlot(0, manualSpindexerSlot);
            //     if (firstOccupied >= 0) {
            //         flywheelSpinning = true;
            //         stateTimer.reset();
            //         currentSpindexerSlot = firstOccupied;
            //         if (hasVisionTarget()) {
            //             manualShootMode = false;
            //             shootState = ShootState.ALIGNING;
            //         } else {
            //             manualShootMode = true;
            //             shootState = ShootState.INDEXING;
            //         }
            //     }
            //     // No balls loaded → don't start (save flywheel wear)
            // } else {
            // Fallback: start from slot 0 (always use this for testing)
            flywheelSpinning = true;
            stateTimer.reset();
            if (hasVisionTarget()) {
                manualShootMode = false;
                shootState = ShootState.ALIGNING;
            } else {
                manualShootMode = true;
                shootState = ShootState.INDEXING;
                currentSpindexerSlot = 0;
            }
            // }
        }
        if (!shootBumper && shootState != ShootState.IDLE && shootState != ShootState.DONE) {
            flywheelSpinning = false;
            manualSpindexerSlot = currentSpindexerSlot;
            shootState = ShootState.IDLE;
        }
        prevShootBumper = shootBumper;

        if (gamepad1.left_bumper || gamepad2.left_bumper) {
            flywheelSpinning = false;
            manualSpindexerSlot = currentSpindexerSlot;
            shootState = ShootState.IDLE;
        }

        // SAFETY: Kick plate ONLY engages in shoot KICKER_UP state when spindexer at shoot positions (0.08, 0.48, 0.89). Never during intake or spindexer cycling - would entangle.
        if (servoArm != null) {
            if (intakeRunning || intakeCoastActive || shootState != ShootState.KICKER_UP) {
                servoArm.setPosition(KICKER_DOWN_POS);
            }
        }

        if (shooterOne != null && shootertwo != null && flywheelSpinning) {
            if (manualShootMode) {
                shooterOne.setPower(MANUAL_SHOOT_POWER);
                shootertwo.setPower(MANUAL_SHOOT_POWER);
            } else {
                // Vision distance → anchor table → RPM and hood so shot matches goal distance
                double vel = computeTargetVelocity();
                shooterOne.setVelocity(vel);
                shootertwo.setVelocity(vel);
            }
        } else if (shooterOne != null && shootertwo != null) {
            shooterOne.setPower(0);
            shootertwo.setPower(0);
        }

        if (shootState == ShootState.IDLE || shootState == ShootState.DONE) {
            if (servoArm != null) servoArm.setPosition(KICKER_DOWN_POS);
            return;
        }

        if (servoArm == null || spindexer == null) return;

        switch (shootState) {
            case ALIGNING:
                hoodPosition = computeHoodFromDistance();
                updateHoodServos();
                if (isAligned() || stateTimer.seconds() > 5) {
                    // COMMENTED OUT: Smart sensor ball detection - testing sequential cycling
                    // if (hasSmartSensors()) {
                    //     int occupied = findNextOccupiedSlot(0, currentSpindexerSlot);
                    //     if (occupied < 0) {
                    //         // No balls — abort shoot
                    //         flywheelSpinning = false;
                    //         shootState = ShootState.IDLE;
                    //     } else {
                    //         currentSpindexerSlot = occupied;
                    //         shootState = ShootState.INDEXING;
                    //         stateTimer.reset();
                    //     }
                    // } else {
                    // Always use fallback: sequential cycling
                    shootState = ShootState.INDEXING;
                    currentSpindexerSlot = 0;
                    stateTimer.reset();
                    // }
                }
                break;

            case INDEXING:
                boolean bBtnIdx = gamepad1.b || gamepad2.b;
                if (bBtnIdx && !prevB) {
                    advanceShootSlot();
                    stateTimer.reset();
                }
                prevB = bBtnIdx;
                spindexer.setPosition(getSpindexerShootPos(currentSpindexerSlot));
                if (manualShootMode && stateTimer.seconds() < MANUAL_SHOOT_SPINUP_SEC) break;
                if (!isKickerSafeToRun()) break;
                int indexMs = manualShootMode ? MANUAL_INDEX_MS : SPINDEXER_INDEX_MS;
                if (stateTimer.milliseconds() >= indexMs) {
                    shootState = ShootState.KICKER_UP;
                    stateTimer.reset();
                }
                break;

            case KICKER_UP:
                // Only place kick plate engages: spindexer already at shoot positions (0.08, 0.48, 0.89) from INDEXING
                spindexer.setPosition(getSpindexerShootPos(currentSpindexerSlot));
                servoArm.setPosition(KICKER_UP_POS);
                int kickUpMs = manualShootMode ? MANUAL_KICKER_UP_MS : KICKER_UP_MS;
                if (stateTimer.milliseconds() >= kickUpMs) {
                    shootState = ShootState.KICKER_DOWN;
                    stateTimer.reset();
                }
                break;

            case KICKER_DOWN:
                servoArm.setPosition(KICKER_DOWN_POS);
                spindexer.setPosition(getSpindexerShootPos(currentSpindexerSlot));
                int kickDownMs = manualShootMode ? MANUAL_KICKER_DOWN_MS : KICKER_DOWN_MS;
                if (stateTimer.milliseconds() >= kickDownMs) {
                    // COMMENTED OUT: Smart sensor ball detection - testing sequential cycling
                    // if (hasSmartSensors()) {
                    //     // Find next occupied slot (skip just-kicked slot)
                    //     int nextOccupied = findNextOccupiedSlot(
                    //             (currentSpindexerSlot + 1) % 3, currentSpindexerSlot);
                    //     if (nextOccupied < 0) {
                    //         // All slots empty — done shooting
                    //         flywheelSpinning = false;
                    //         shootState = ShootState.IDLE;
                    //     } else {
                    //         currentSpindexerSlot = nextOccupied;
                    //         shootState = ShootState.INDEXING;
                    //         stateTimer.reset();
                    //     }
                    // } else {
                    // Always use fallback: sequential cycling with revolution tracking (5-turn servo)
                    advanceShootSlot();
                    shootState = ShootState.INDEXING;
                    stateTimer.reset();
                    // }
                }
                break;

            default:
                break;
        }
        if (shootState != ShootState.IDLE && shootState != ShootState.DONE) prevB = gamepad1.b || gamepad2.b;
    }

    private boolean isKickerSafeToRun() {
        return shootState != ShootState.KICKER_UP && shootState != ShootState.KICKER_DOWN;
    }

    /** Spindexer position for SHOOTING — per-slot calibrated base + revolution offset. */
    private double getSpindexerShootPos(int slot) {
        double base;
        if (slot == 0) base = SPINDEXER_SHOOT_0;
        else if (slot == 1) base = SPINDEXER_SHOOT_1;
        else base = SPINDEXER_SHOOT_2;
        return Math.max(0.0, Math.min(1.0, base + spindexerRevolution * SPINDEXER_REV_OFFSET));
    }

    /** Spindexer position for INTAKE — per-slot calibrated base + revolution offset. */
    private double getSpindexerIntakePos(int slot) {
        double base;
        if (slot == 0) base = SPINDEXER_INTAKE_0;
        else if (slot == 1) base = SPINDEXER_INTAKE_1;
        else base = SPINDEXER_INTAKE_2;
        return Math.max(0.0, Math.min(1.0, base + spindexerRevolution * SPINDEXER_REV_OFFSET));
    }

    /**
     * Advance the spindexer by one slot in physical order (1→0→2, forward only).
     * Physical order: slot 1 (lowest) → slot 0 (middle) → slot 2 (highest).
     * When reaching max revolution (rev 2, slot 1), wraps back to rev 0, slot 1.
     * Updates intakeCycleStep and spindexerRevolution.
     */
    private void advanceSpindexerSlot() {
        // Physical order: 1→0→2 (slot indices)
        // Map: 1→0→2→1→0→2...
        if (intakeCycleStep == 1) {
            intakeCycleStep = 0;  // 1 → 0
        } else if (intakeCycleStep == 0) {
            intakeCycleStep = 2;  // 0 → 2
        } else { // intakeCycleStep == 2
            intakeCycleStep = 1;  // 2 → 1
            spindexerRevolution++;  // Completed one full cycle
            if (spindexerRevolution > SPINDEXER_MAX_FWD_REV) {
                // Hit max (rev 2, slot 1) - wrap back to start
                spindexerRevolution = 0;
                intakeCycleStep = 1;  // Back to rev 0, slot 1
            }
        }
    }

    /**
     * Advance the shoot spindexer slot by one in the forward direction.
     * During shooting, revolution stays at 0 to keep positions aligned with kicker plate.
     * Updates currentSpindexerSlot only (does NOT modify spindexerRevolution).
     */
    private void advanceShootSlot() {
        currentSpindexerSlot = (currentSpindexerSlot + 1) % 3;
        // Note: spindexerRevolution remains at 0 during shooting for consistent alignment
    }

    // ======================== SMART SPINDEXER ========================
    // The 3 color sensors (cs3, colorSenor, cs2) are FIXED on the robot frame.
    // As the turntable rotates, different logical slots pass under different sensors.
    // Sensor index: 0=cs3, 1=colorSenor, 2=cs2
    // Reference (activeSlot=0): sensor 0→slot 0, sensor 1→slot 1, sensor 2→slot 2
    // When activeSlot=N, the mapping rotates by N (direction set by SENSOR_ROTATION_SIGN).

    /** True if all 3 color sensors are configured (smart spindexer available). */
    private boolean hasSmartSensors() {
        return cs2 != null && cs3 != null && colorSenor != null;
    }

    /** Read raw sensor by index: 0=cs3, 1=colorSenor, 2=cs2. Returns true if ball detected. */
    private boolean sensorDetectsBall(int sensorIndex) {
        ColorSensor sensor;
        if (sensorIndex == 0) sensor = cs3;
        else if (sensorIndex == 1) sensor = colorSenor;
        else sensor = cs2;
        if (sensor == null) return false;
        return (sensor.red() + sensor.green() + sensor.blue()) >= BALL_MIN_BRIGHTNESS;
    }

    /**
     * Check if a logical slot has a ball, accounting for turntable rotation.
     * @param slot       logical slot to check (0, 1, or 2)
     * @param activeSlot the slot currently at the intake/kicker position
     *                   (determines turntable orientation → sensor mapping)
     */
    private boolean isSlotOccupied(int slot, int activeSlot) {
        if (!hasSmartSensors()) return false;
        int sensorIndex;
        if (SENSOR_ROTATION_SIGN > 0) {
            // sensor s sees slot (s + activeSlot) % 3
            // → to read slot n, use sensor (n - activeSlot + 3) % 3
            sensorIndex = ((slot - activeSlot) % 3 + 3) % 3;
        } else {
            // sensor s sees slot (s - activeSlot + 3) % 3
            // → to read slot n, use sensor (n + activeSlot) % 3
            sensorIndex = (slot + activeSlot) % 3;
        }
        return sensorDetectsBall(sensorIndex);
    }

    /** Find the next empty slot starting from startSlot (wraps around). Returns -1 if all full. */
    private int findNextEmptySlot(int startSlot, int activeSlot) {
        for (int i = 0; i < 3; i++) {
            int slot = (startSlot + i) % 3;
            if (!isSlotOccupied(slot, activeSlot)) return slot;
        }
        return -1;
    }

    /** Find the next occupied slot starting from startSlot (wraps around). Returns -1 if all empty. */
    private int findNextOccupiedSlot(int startSlot, int activeSlot) {
        for (int i = 0; i < 3; i++) {
            int slot = (startSlot + i) % 3;
            if (isSlotOccupied(slot, activeSlot)) return slot;
        }
        return -1;
    }

    private double computeTargetVelocity() {
        if (!hasVisionTarget()) return targetVelocity;
        double dist = getVisionDistanceInches();
        if (dist >= FAR_DISTANCE_THRESHOLD_IN)
            return VELOCITY_FAR_SIDE_MAX * BACKSPIN_RPM_FACTOR;
        double vel = interpolateFromAnchors(dist, SHOOTER_ANCHOR_DIST_IN, SHOOTER_ANCHOR_VELOCITY);
        vel *= BACKSPIN_RPM_FACTOR;
        if (vel < VELOCITY_MIN) vel = VELOCITY_MIN;
        if (vel > VELOCITY_MAX) vel = VELOCITY_MAX;
        return vel;
    }

    private double computeHoodFromDistance() {
        if (!hasVisionTarget()) return hoodPosition;
        double dist = getVisionDistanceInches();
        double h = interpolateFromAnchors(dist, SHOOTER_ANCHOR_DIST_IN, SHOOTER_ANCHOR_HOOD);
        h += BACKSPIN_HOOD_OFFSET;
        if (h < 0) h = 0;
        if (h > 1) h = 1;
        return h;
    }

    /**
     * Map internal hoodPosition (0.0=low, 1.0=high) to 5-turn servo position.
     * Inverted: hoodPosition 0.0 → servo 0.500 (bottom), hoodPosition 1.0 → servo 0.180 (top).
     */
    private double mapHoodToServoPosition(double hoodPos) {
        // Clamp to [0, 1] first
        hoodPos = Math.max(0.0, Math.min(1.0, hoodPos));
        // Inverted linear mapping: 0.0 → 0.500, 1.0 → 0.180
        return HOOD_POS_BOTTOM - (hoodPos * (HOOD_POS_BOTTOM - HOOD_POS_TOP));
    }

    /** Linear interpolation from anchor arrays. distIn = corrected distance; arrays same length. */
    private double interpolateFromAnchors(double distIn, double[] distAnchors, double[] valueAnchors) {
        if (distAnchors == null || valueAnchors == null || distAnchors.length != valueAnchors.length || distAnchors.length == 0)
            return valueAnchors != null && valueAnchors.length > 0 ? valueAnchors[0] : 0;
        if (distIn <= distAnchors[0]) return valueAnchors[0];
        if (distIn >= distAnchors[distAnchors.length - 1]) return valueAnchors[valueAnchors.length - 1];
        for (int i = 0; i < distAnchors.length - 1; i++) {
            if (distIn >= distAnchors[i] && distIn <= distAnchors[i + 1]) {
                double t = (distIn - distAnchors[i]) / (distAnchors[i + 1] - distAnchors[i]);
                return valueAnchors[i] + t * (valueAnchors[i + 1] - valueAnchors[i]);
            }
        }
        return valueAnchors[valueAnchors.length - 1];
    }

    private boolean isAligned() {
        if (!hasVisionTarget()) return false;
        double tx = getVisionTxDegrees();
        double txErr = (Math.abs(tx) < 999) ? (tx - ALIGNMENT_TX_OFFSET_DEG) : 999;
        return Math.abs(txErr) <= ALIGNMENT_TOLERANCE_DEG;
    }

    /** Wrap angle in degrees to -180..180 for heading error. */
    private double wrapDeg(double d) {
        while (d > 180) d -= 360;
        while (d < -180) d += 360;
        return d;
    }

    private void updateHood() {
        if (gamepad1.dpad_left || gamepad2.dpad_left) hoodPosition = Math.max(0, hoodPosition - 0.02);
        if (gamepad1.dpad_right || gamepad2.dpad_right) hoodPosition = Math.min(1, hoodPosition + 0.02);
        if (shootState != ShootState.ALIGNING) {
            updateHoodServos();
        }
    }

    private void updateHoodServos() {
        if (hoodOne == null || hood == null) return;
        boolean inShootSequence = shootState == ShootState.ALIGNING || shootState == ShootState.INDEXING
                || shootState == ShootState.KICKER_UP || shootState == ShootState.KICKER_DOWN;
        if (inShootSequence) hoodPosition = computeHoodFromDistance();
        double servoPos = mapHoodToServoPosition(hoodPosition);
        hoodOne.setPosition(servoPos);
        // Second servo moves opposite: when hoodOne is at bottom (0.500), hood is at top (0.180)
        hood.setPosition(HOOD_POS_TOP + HOOD_POS_BOTTOM - servoPos);
    }

    /** Green ball: green clearly dominant (e.g. R29 G68 B58). No ball = white only. */
    private boolean isGreenBallDetected() {
        if (colorSenor == null) return false;
        int r = colorSenor.red(), g = colorSenor.green(), b = colorSenor.blue();
        int total = r + g + b;
        if (total < BALL_MIN_BRIGHTNESS) return false;
        return g > r + BALL_GREEN_DOMINANCE && g > b + BALL_GREEN_DOMINANCE;
    }

    /** Purple ball: red and blue high, green not dominant (e.g. R38 G58 B66). */
    private boolean isPurpleBallDetected() {
        if (colorSenor == null) return false;
        int r = colorSenor.red(), g = colorSenor.green(), b = colorSenor.blue();
        int total = r + g + b;
        if (total < BALL_MIN_BRIGHTNESS) return false;
        return r > BALL_PURPLE_MIN && b > BALL_PURPLE_MIN && (r + b) > g + BALL_GREEN_DOMINANCE;
    }

    /** LED: shooter aligning = blink, aligned/emptying = solid; else AprilTag blink, ball color, red. */
    private void updateLedFromColor() {
        if (led == null) return;

        boolean inShootSequence = shootState != ShootState.IDLE && shootState != ShootState.DONE;
        boolean aligning = shootState == ShootState.ALIGNING && hasVisionTarget() && !isAligned();
        boolean alignedOrEmptying = (shootState == ShootState.ALIGNING && isAligned())
                || shootState == ShootState.INDEXING || shootState == ShootState.KICKER_UP || shootState == ShootState.KICKER_DOWN;

        if (inShootSequence && aligning) {
            boolean blinkOn = ((int) (ledBlinkTimer.milliseconds() / LED_BLINK_MS)) % 2 == 0;
            led.setPosition(blinkOn ? LED_POS_YELLOW : LED_POS_OFF);
            return;
        }
        if (inShootSequence && alignedOrEmptying) {
            led.setPosition(LED_POS_YELLOW);  // solid = aligned, emptying spindexer
            return;
        }

        boolean goalPostDetected = hasVisionTarget() || hasAnyGoalTarget();
        boolean greenBall = isGreenBallDetected();
        boolean purpleBall = isPurpleBallDetected();

        // Goal post detected: blink same color as ball (green/purple) or yellow if no ball
        if (goalPostDetected) {
            boolean blinkOn = ((int) (ledBlinkTimer.milliseconds() / LED_BLINK_MS)) % 2 == 0;
            double goalColor = greenBall ? LED_POS_GREEN_BALL : (purpleBall ? LED_POS_PURPLE_BALL : LED_POS_YELLOW);
            led.setPosition(blinkOn ? goalColor : LED_POS_OFF);
            return;
        }
        if (greenBall) {
            led.setPosition(LED_POS_GREEN_BALL);
            return;
        }
        if (purpleBall) {
            led.setPosition(LED_POS_PURPLE_BALL);
            return;
        }
        led.setPosition(LED_POS_RED);
    }

    private void updateTelemetry() {
        telemetry.addData("Status", "Running");
        telemetry.addData("Alliance", "BLUE");
        try { telemetry.addData("Battery", String.format("%.2f V", hardwareMap.voltageSensor.iterator().next().getVoltage())); } catch (Exception e) {}
        telemetry.addData("Shoot", shootState.toString());
        telemetry.addData("Mode", manualShootMode ? "MANUAL (80%)" : "AUTO (vision)");
        telemetry.addData("Flywheel", flywheelSpinning ? "ON" : "OFF");
        telemetry.addData("Target Vel", (int) computeTargetVelocity());
        telemetry.addData("Drive front", driveOrientationIntakeFront ? "INTAKE (LT slight)" : "SHOOTER (RT/RB slight)");
        telemetry.addData("Indexing (B)", intakeCycleToggle ? "ON (press B again to stop)" : "OFF (press B to run)");

        // Slot occupancy from smart sensors (fixed sensors + rotation mapping)
        if (hasSmartSensors()) {
            // Determine current active slot for sensor mapping
            int activeSlot = manualSpindexerSlot;
            if (intakeRunning || intakeCoastActive) activeSlot = intakeCycleStep;
            else if (shootState != ShootState.IDLE && shootState != ShootState.DONE) activeSlot = currentSpindexerSlot;
            String s0 = isSlotOccupied(0, activeSlot) ? "BALL" : "empty";
            String s1 = isSlotOccupied(1, activeSlot) ? "BALL" : "empty";
            String s2 = isSlotOccupied(2, activeSlot) ? "BALL" : "empty";
            telemetry.addData("Slots 0|1|2", "[" + s0 + "] [" + s1 + "] [" + s2 + "]");
            // Raw sensor values for calibration (sensor 0=cs3, 1=color, 2=cs2)
            telemetry.addData("Raw sensors",
                    "cs3=" + (cs3.red()+cs3.green()+cs3.blue())
                            + " cs=" + (colorSenor.red()+colorSenor.green()+colorSenor.blue())
                            + " cs2=" + (cs2.red()+cs2.green()+cs2.blue()));
            telemetry.addData("Active slot", activeSlot);
            telemetry.addData("Rotation sign", SENSOR_ROTATION_SIGN > 0 ? "+1" : "-1");
        }
        if (cs2 == null || cs3 == null) {
            telemetry.addData("Smart sensors", "cs2=" + (cs2 != null ? "OK" : "MISSING")
                    + " cs3=" + (cs3 != null ? "OK" : "MISSING"));
        }

        if ((gamepad1.a || gamepad2.a) && !prevG2A) {
            velocityPreset = (velocityPreset + 1) % 3;
            targetVelocity = velocityPreset == 0 ? SHOOTER_ANCHOR_VELOCITY[0] : velocityPreset == 1 ? SHOOTER_ANCHOR_VELOCITY[1] : SHOOTER_ANCHOR_VELOCITY[2];
        }
        prevG2A = gamepad1.a || gamepad2.a;

        // --- LimeLight / Goal post status (Driver Station) ---
        boolean anyGoal = hasAnyGoalTarget();
        int closestTagId = getClosestGoalTagId();
        double closestDist = getClosestGoalDistanceInches();
        double closestTx = getClosestGoalTx();
        if (limelightEnabled && limelight != null) {
            LLResult llRes = limelight.getLatestResult();
            telemetry.addData("LimeLight", llRes != null && llRes.isValid() ? "CONNECTED" : "NO RESULT");
            if (llRes != null && llRes.isValid()) {
                telemetry.addData("LimeLight tx", String.format("%.1f deg", llRes.getTx()));
                telemetry.addData("LimeLight ty", String.format("%.1f deg", llRes.getTy()));
                telemetry.addData("LimeLight ta", String.format("%.0f%%", llRes.getTa()));
            }
        } else {
            telemetry.addData("LimeLight", "not connected");
        }
        if (anyGoal && closestTagId >= 0) {
            String which = isTagIdInAlliance(closestTagId) ? "alliance" : "other";
            telemetry.addData("Goal post", "DETECTED (Tag " + closestTagId + ", " + String.format("%.1f in", closestDist) + ", " + which + ")");
            telemetry.addData("Goal tx", String.format("%.1f deg", closestTx));
        } else {
            telemetry.addData("Goal post", "NOT DETECTED");
        }

        if (hasVisionTarget()) {
            double dist = getVisionDistanceInches();
            double tx = getVisionTxDegrees();
            double txErr = (Math.abs(tx) < 999) ? (tx - ALIGNMENT_TX_OFFSET_DEG) : 0;
            telemetry.addData("Vision (shoot)", limelightEnabled ? "LimeLight" : "AprilTag");
            telemetry.addData("Alliance target", "YES");
            telemetry.addData("Distance", String.format("%.1f in (ty-based)", dist));
            telemetry.addData("Computed vel", String.format("%.0f tks/s", computeTargetVelocity()));
            telemetry.addData("Computed hood", String.format("%.3f", computeHoodFromDistance()));
            telemetry.addData("tx", String.format("%.1f deg (0=aligned)", tx));
            if (Math.abs(txErr) <= ALIGNMENT_TOLERANCE_DEG) telemetry.addData(">>>", "ALIGNED - SHOOT");
            else if (Math.abs(tx) < 999) telemetry.addData(">>>", txErr > 0 ? "STRAFE LEFT" : "STRAFE RIGHT");
            int tagId = getVisionTagId();
            if (tagId >= 0) telemetry.addData("Tag ID", tagId);
        } else {
            telemetry.addData("Vision (shoot)", "No alliance target");
            if (limelightEnabled && limelight != null) {
                LLResult llRes = limelight.getLatestResult();
                if (llRes == null) telemetry.addLine("LimeLight: result null (wait or check USB)");
                else if (!llRes.isValid()) telemetry.addLine("LimeLight: set Pipeline 0 = Fiducial Markers");
                else if (!anyGoal) telemetry.addLine("LimeLight: no goal tag in view (show tag 1-6)");
            }
        }

        if (colorSenor != null) {
            int r = colorSenor.red(), g = colorSenor.green(), b = colorSenor.blue();
            int total = r + g + b;
            boolean greenOk = total >= BALL_MIN_BRIGHTNESS && g > r + BALL_GREEN_DOMINANCE && g > b + BALL_GREEN_DOMINANCE;
            boolean purpleOk = total >= BALL_MIN_BRIGHTNESS && r > BALL_PURPLE_MIN && b > BALL_PURPLE_MIN && (r + b) > g + BALL_GREEN_DOMINANCE;
            String ballStr = greenOk ? "GREEN" : (purpleOk ? "PURPLE" : "none");
            telemetry.addData("Ball (R G B)", "%d %d %d total=%d", r, g, b, total);
            telemetry.addData("Ball detected", ballStr);
            telemetry.addLine("(REV config: colorSenor or colorSensor; enable LED in config if no detect)");
        } else telemetry.addData("Color", "No sensor (REV config: colorSenor or colorSensor)");
        if (led != null) {
            if (hasVisionTarget() || hasAnyGoalTarget()) telemetry.addData("LED", "BLINK (goal = ball color or yellow)");
            else if (isGreenBallDetected()) telemetry.addData("LED", "GREEN (ball)");
            else if (isPurpleBallDetected()) telemetry.addData("LED", "PURPLE (ball)");
            else telemetry.addData("LED", "OFF (no ball)");
        }
        boolean shootBtn = gamepad1.right_bumper || gamepad2.right_bumper || gamepad1.right_trigger > 0.5 || gamepad2.right_trigger > 0.5;
        telemetry.addData("Shoot (RB/RT)", shootBtn ? "HELD" : "---");

        if (pinpoint != null && pinpointEnabled) {
            telemetry.addData("Pinpoint", "ON");
            telemetry.addData("Pinpoint X", String.format("%.1f in", pinpoint.getPosX(DistanceUnit.INCH)));
            telemetry.addData("Pinpoint Y", String.format("%.1f in", pinpoint.getPosY(DistanceUnit.INCH)));
            telemetry.addData("Pinpoint Heading", String.format("%.1f deg", pinpoint.getHeading(AngleUnit.DEGREES)));
        } else {
            telemetry.addData("Pinpoint", "not configured");
        }
        telemetry.update();
    }

    private void stopAll() {
        if (leftFront != null) leftFront.setPower(0);
        if (rightFront != null) rightFront.setPower(0);
        if (leftBack != null) leftBack.setPower(0);
        if (rightBack != null) rightBack.setPower(0);
        if (intake != null) intake.setPower(0);
        if (shooterOne != null) shooterOne.setPower(0);
        if (shootertwo != null) shootertwo.setPower(0);
        if (servoArm != null) servoArm.setPosition(KICKER_DOWN_POS);
    }
}