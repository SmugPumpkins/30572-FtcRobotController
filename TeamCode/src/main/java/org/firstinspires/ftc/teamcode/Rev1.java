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
 *
 * VISION: LimeLight 3A primary (tx, ty, ta, fiducial); Webcam AprilTag fallback
 * LED: Color sensor -> ball color (green/purple) displayed on LED; flashes red when 3 balls loaded
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
 *   Left Stick:         X=forward/back, Y=strafe. Front = shooting side.
 *   Right Stick X:      Rotate
 *   Left Trigger (LT):  Intake (hold) - full RPM, spindexer cycles with smart sensors
 *   LT + A:             Set intake direction CW (1->0->2)
 *   LT + B:             Set intake direction CCW (2->0->1)
 *   LT + D-pad Up:      Intake REVERSE (release stuck balls)
 *   Right Trigger (RT): Auto shoot (hold) - vision-assisted, all balls in hardware order
 *   RT + A:             Auto shoot - PURPLE balls only
 *   RT + B:             Auto shoot - GREEN balls only
 *   Right Bumper (RB):  Manual shoot (hold) - no vision
 *   RB + A:             Manual shoot - near (50% power, hood near)
 *   RB + B:             Manual shoot - far (100% power, hood far)
 *   Left Bumper (LB):   Shooter OFF
 *   Y:                  AUTO-DRIVE toward AprilTag (hold = rotate + drive forward)
 *   D-pad Left:         Hood down
 *   D-pad Right:        Hood up
 *   X:                  IMU yaw reset
 *
 * (G2 same as G1 if second gamepad connected)
 */

@TeleOp(name = "Rev1", group = "Driver")
public class Rev1 extends LinearOpMode {

    // === DRIVE ===
    public static double MAX_SPEED = 1.0;
    public static double PRECISION_SPEED = 0.5;
    public static double DRIVE_CURVE_EXPONENT = 2.0;
    public static double STRAFE_MULTIPLIER = 0.9;
    public static double ROTATE_MULTIPLIER = 1.0;
    public static double DEAD_ZONE = 0.08;
    public static double INTAKE_POWER = 0.8;

    // === SHOOTER - direct ty-based mapping (no intermediate distance calculation) ===
    // goBILDA 5202 bare 6000RPM motor, 28 CPR encoder, 1:1 ratio
    // Max tks/s: ~2800 @ 12V, ~3080 @ 13.2V (fresh battery)
    public static double VELOCITY_MIN = 800;
    public static double VELOCITY_MAX = 2800;   // motor physical limit @ 12V (6000RPM × 28CPR / 60)
    public static final double MOTOR_ENCODER_CPR = 28.0;  // encoder counts per motor revolution
    /**
     * Direct ty -> velocity/hood mapping.  CALIBRATED anchor: ty≈-7 @ 68" real → 2693 tks/s, hood 0.05.
     * Physics notes (ball must rise 29" from 9.625" flywheel to 38.7" goal):
     *   Close range (<40"): steep hood angle needed (ball goes UP a lot in short distance)
     *   Mid range (50-70"): moderate speed, hood mostly flat
     *   Far range (>80"): near motor limit, hood rises slightly for optimal arc
     */
    public static double[] SHOOTER_ANCHOR_TY     = { -12.5,   -8.0,   -6.0,    -2.6,    -0.4,     4.0   };
    //                               approx dist:   ~117"   ~83"   ~74"    ~61"   ~55"   ~45"
    public static double[] SHOOTER_TY_VELOCITY    = { 1793,  1193,  1193,  1193,  1093,  1193 };
    public static double[] SHOOTER_TY_HOOD        = { 0.65,  0.05,  0.20,  0.30,  0.40,  0.50 };
    public static double VELOCITY_DEFAULT = 1400;   // 3000 RPM close-range fallback (3000×28/60)
    /** Far-side: ty below this threshold -> motor max power (saturated). */
    public static double FAR_TY_THRESHOLD = -14.0;
    public static double VELOCITY_FAR_SIDE_MAX = 2800.0;  // motor ceiling, not arbitrary

    // === HOOD (5-turn torque motors, inverted: lower value = higher position) ===
    public static double HOOD_POS_TOP = 0.180;
    public static double HOOD_POS_BOTTOM = 0.500;

    // === MANUAL SHOOT (RB) — fixed safe close-range velocity ===
    public static double MANUAL_SHOOT_HOOD = 0.55;   // close-range hood (from anchor table, ty=10°/~30")

    // === SPINDEXER: goBILDA 5-turn servo (calibrated absolute positions) ===
    // !! WARNING: CW-ONLY !! This servo has significant backlash. NEVER cycle CCW.
    // Always advance S1->S0->S2 (CW), then REWIND back to S1. Proven 2026-02-24.
    // Physical order per rotation: Slot 1 (lowest) -> Slot 0 (middle) -> Slot 2 (highest)
    public static double SPINDEXER_SHOOT_0 = 0.119;
    public static double SPINDEXER_SHOOT_1 = 0.054;
    public static double SPINDEXER_SHOOT_2 = 0.184;
    public static double SPINDEXER_INTAKE_OFFSET = -0.007;
    public static double SPINDEXER_INTAKE_0 = 0.112;
    public static double SPINDEXER_INTAKE_1 = 0.047;
    public static double SPINDEXER_INTAKE_2 = 0.177;
    /**
     * Shoot positions in CW cycling order (S1->S0->S2). R1 only — rewind after S2.
     * !! DO NOT add more rotations. CW-only within R1 eliminates backlash misalignment.
     */
    public static double[] SPINDEXER_SHOOT_ALL = {
            0.054, 0.119, 0.184    // Rotation 1: S1, S0, S2 (CW-only, rewind after S2)
    };
    /** Logical slot index for each position in SPINDEXER_SHOOT_ALL. */
    public static int[] SPINDEXER_SLOT_MAP = {1, 0, 2};
    /**
     * Sensor rotation mapping sign. The 3 color sensors are FIXED on the robot frame;
     * +1: sensor[s] sees slot (s + activeSlot) % 3
     * -1: sensor[s] sees slot (s - activeSlot + 3) % 3
     */
    public static int SENSOR_ROTATION_SIGN = 1;
    public static int SPINDEXER_INDEX_MS = 630;   // calibrated: servo settle before kick (CW-only)
    public static int SPINDEXER_INTAKE_CYCLE_MS = 450;
    public static double INTAKE_COAST_SEC = 3.0;

    // === KICKER ===
    public static double KICKER_UP_POS = 0.060;
    public static double KICKER_DOWN_POS = 0.330;
    public static int KICKER_UP_MS = 110;   // calibrated push duration
    public static int KICKER_DOWN_MS = 300;  // SAFE: full retract before spindexer moves

    // === ALIGNMENT ===
    public static double ALIGNMENT_TOLERANCE_DEG = 2.0;
    public static double ALIGNMENT_TX_OFFSET_DEG = 0.0;
    public static double ALIGNMENT_CORRECTION_SIGN = -1.0;
    public static double ALIGNMENT_STRAFE_GAIN = 0.03;
    public static double ALIGNMENT_ROTATE_GAIN = 0.04;
    public static double ALIGNMENT_TIMEOUT_SEC = 1.5;   // reduced from 5.0 - shoot ASAP
    public static double AUTO_DRIVE_ROTATE_GAIN = 0.02;
    public static double AUTO_DRIVE_FORWARD_GAIN = 0.015;
    public static double INTAKE_ORIENTATION_TRIGGER_THRESHOLD = 0.2;

    // === DEFENSIVE DRIVE ===
    public static boolean DEFENSIVE_LOCK_ENABLED = true;
    public static double DEFENSIVE_LOCK_KP_X = 0.03;
    public static double DEFENSIVE_LOCK_KP_Y = 0.03;
    public static double DEFENSIVE_LOCK_KP_HEADING = 0.02;
    public static double DEFENSIVE_LOCK_DEADBAND_IN = 0.5;
    public static double DEFENSIVE_LOCK_DEADBAND_DEG = 2.0;
    public static double DEFENSIVE_LOCK_MAX_CORRECTION = 0.4;

    // === MANUAL SHOOT TIMING (override when no AprilTag) ===
    public static double MANUAL_SHOOT_SPINUP_SEC = 1.0;
    public static int MANUAL_INDEX_MS = 630;   // same as auto (calibrated)
    public static int MANUAL_KICKER_UP_MS = 110;   // same as auto (calibrated)
    public static int MANUAL_KICKER_DOWN_MS = 300;  // SAFE: full retract before spindexer moves

    // === LED ===
    public static double LED_POS_OFF = 0.0;
    public static double LED_POS_RED = 0.35;
    public static double LED_POS_YELLOW = 0.388;
    public static double LED_POS_GREEN_BALL = 0.500;
    public static double LED_POS_PURPLE_BALL = 0.666;
    public static double LED_POS_WHITE = 0.722;
    public static int LED_BLINK_MS = 250;
    public static int BALL_MIN_BRIGHTNESS = 300;   // raised from 50 — ambient light caused false positives
    public static int BALL_GREEN_DOMINANCE = 30;    // raised from 5 — need clear color dominance
    public static int BALL_PURPLE_MIN = 80;          // raised from 30

    // === GOAL POST TAGS ===
    public static int[] BLUE_TAG_IDS = { 20 };
    public static int[] RED_TAG_IDS  = { 24 };
    public static double DISTANCE_CORRECTION = 0.95;
    // Camera calibration (measured values)
    public static double VISION_TAG_HEIGHT_IN = 38.7;           // AprilTag center height (CAD: 0.984m = 38.7")
    public static double VISION_CAMERA_HEIGHT_IN = 14.875;     // 14 7/8" lens height from floor (MEASURED)
    public static double VISION_CAMERA_TILT_DEG = 24.0;        // 24 degrees upward tilt (MEASURED)
    public static double FLYWHEEL_HEIGHT_IN = 9.625;            // 9 5/8" flywheel center from floor (MEASURED)
    public static double CAMERA_FLYWHEEL_HORIZONTAL_IN = 6.0;  // horizontal offset lens to flywheel (MEASURED)

    // === BALL COLOR TRACKING ===
    enum BallColor { EMPTY, GREEN, PURPLE }

    // === HARDWARE ===
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private DcMotor intake;
    private IMU imu;
    private DcMotorEx shooterOne, shootertwo;
    private Servo hoodOne, hood;
    private Servo spindexer, servoArm;
    private ColorSensor colorSenor;   // sensor 1 (fixed)
    private ColorSensor cs2;          // sensor 2 (fixed)
    private ColorSensor cs3;          // sensor 3 (fixed)
    private Servo led;
    private Limelight3A limelight;
    private boolean limelightEnabled = false;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private boolean visionEnabled = false;
    private GoBildaPinpointDriver pinpoint = null;
    private boolean pinpointEnabled = false;

    // === STATE ===
    private double targetVelocity = VELOCITY_DEFAULT;
    private boolean intakeRunning = false;
    private double hoodPosition = 0.5;
    private boolean flywheelSpinning = false;
    private boolean prevG1LB, prevG2RB, prevG2LB;
    private boolean prevA = false, prevB = false;
    private int manualSpindexerSlot = 1;
    private boolean intakeCoastActive = false;
    private ElapsedTime intakeCoastTimer = new ElapsedTime();
    private int intakeCycleStep = 0;
    private int spindexerPosIndex = 0;       // index into SPINDEXER_SHOOT_ALL for intake
    private ElapsedTime intakeCycleTimer = new ElapsedTime();
    private ElapsedTime ledBlinkTimer = new ElapsedTime();
    private boolean driveOrientationIntakeFront = false;

    // Intake direction: true = CW (1->0->2), false = CCW (2->0->1)
    private boolean intakeCW = true;

    // Ball color per slot (updated every loop cycle from sensors)
    private BallColor[] slotBalls = { BallColor.EMPTY, BallColor.EMPTY, BallColor.EMPTY };

    // LED flash state (3 balls loaded -> flash red 3x)
    private boolean ledFlashActive = false;
    private ElapsedTime ledFlashTimer = new ElapsedTime();
    private boolean prevAllSlotsFull = false;

    // Shoot state machine
    private enum ShootState { IDLE, ALIGNING, INDEXING, KICKER_UP, KICKER_DOWN, DONE }
    private ShootState shootState = ShootState.IDLE;
    private boolean manualShootMode = false;
    private int currentSpindexerSlot = 0;
    private int shootPosIndex = 0;           // index into SPINDEXER_SHOOT_ALL for shooting
    private int shootScanCount = 0;          // positions scanned since last kick (color filter)
    private ElapsedTime stateTimer = new ElapsedTime();
    private boolean prevShootRT = false;
    private boolean prevShootRB = false;

    // Shoot color filter: which balls to shoot
    private enum ShootColorMode { ALL, PURPLE_ONLY, GREEN_ONLY }
    private ShootColorMode shootColorMode = ShootColorMode.ALL;

    // Manual shoot preset (simplified: always safe close-range)

    // Defensive drive (position hold during shooting)
    private boolean defensiveLockActive = false;
    private double lockTargetX = 0, lockTargetY = 0, lockTargetHeading = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        initHardware();
        stopAll();

        // Spindexer flush: always start at Slot 1 (home position)
        if (spindexer != null) {
            spindexerPosIndex = 0;
            shootPosIndex = 0;
            manualSpindexerSlot = 1;
            spindexer.setPosition(SPINDEXER_SHOOT_1);
            sleep(500);
        }

        telemetry.addData("Status", "Ready! RT=Auto Shoot  RB=Manual Shoot");
        telemetry.addData("Code Version", 10);
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (pinpoint != null) pinpoint.update();
            updateSlotColors();
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

    // ======================== VISION HELPERS ========================

    private boolean isTagIdInAlliance(int tagId) {
        for (int id : BLUE_TAG_IDS) { if (id == tagId) return true; }
        return false;
    }

    private boolean isTagIdGoal(int tagId) {
        for (int id : BLUE_TAG_IDS) { if (id == tagId) return true; }
        for (int id : RED_TAG_IDS) { if (id == tagId) return true; }
        return false;
    }

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

    /** Distance (inches) to closest goal - used for telemetry and Y-button auto-drive only. */
    private double getClosestGoalDistanceInches() {
        if (limelightEnabled && limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        if (isTagIdGoal(f.getFiducialId())) {
                            double ty = result.getTy();
                            double tyDist = distanceFromTy(ty);
                            if (tyDist > 0) return tyDist;
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

    /** Distance from Limelight ty using trig. Used for telemetry and Y-button auto-drive. */
    private double distanceFromTy(double tyDeg) {
        double heightDiff = VISION_TAG_HEIGHT_IN - VISION_CAMERA_HEIGHT_IN;
        double totalAngleDeg = VISION_CAMERA_TILT_DEG + tyDeg;
        if (totalAngleDeg < 0.5 || totalAngleDeg > 60) return -1;
        double dist = heightDiff / Math.tan(Math.toRadians(totalAngleDeg));
        return Math.max(24, Math.min(120, dist));
    }

    private double getVisionDistanceInches() {
        return getVisionDistanceInchesRaw() * DISTANCE_CORRECTION;
    }

    private double getVisionDistanceInchesRaw() {
        if (limelightEnabled && limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        if (isTagIdInAlliance(f.getFiducialId())) {
                            double ty = result.getTy();
                            double tyDist = distanceFromTy(ty);
                            if (tyDist > 0) return tyDist;
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

    /** Direct ty reading from Limelight (for velocity/hood computation). */
    private double getVisionTyDegrees() {
        if (limelightEnabled && limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        if (isTagIdInAlliance(f.getFiducialId())) return result.getTy();
                    }
                    return 0;
                }
                return result.getTy();
            }
        }
        return 0;
    }

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

    // ======================== INIT ========================

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
            limelight.pipelineSwitch(0);
            limelight.start();
            limelightEnabled = true;
        } catch (Exception e) { limelight = null; limelightEnabled = false; }

        try {
            aprilTag = new AprilTagProcessor.Builder().setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f).build();
            visionPortal = new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).addProcessor(aprilTag).build();
            visionEnabled = true;
        } catch (Exception e) { visionEnabled = false; }
    }

    // ======================== DRIVE ========================

    private void updateDrive() {
        if (gamepad1.x || gamepad2.x) {
            imu.resetYaw();
        }

        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

        // Field-relative driving using IMU
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);
        theta = AngleUnit.normalizeRadians(theta - robotYaw);
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);
        forward = newForward;
        double strafe = newRight;

        // Y button: auto-align to goal
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

        // Auto-strafe during ALIGNING state
        if (shootState == ShootState.ALIGNING && hasVisionTarget()) {
            double tx = getVisionTxDegrees();
            double txErr = (Math.abs(tx) < 999) ? (tx - ALIGNMENT_TX_OFFSET_DEG) : 0;
            if (Math.abs(txErr) > ALIGNMENT_TOLERANCE_DEG) {
                double corr = ALIGNMENT_CORRECTION_SIGN * txErr;
                strafe += corr * ALIGNMENT_STRAFE_GAIN;
                rotate += corr * ALIGNMENT_ROTATE_GAIN;
            }
        }

        // Defensive drive: position hold during shooting
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

        // Input curve
        if (forward != 0) forward = Math.signum(forward) * Math.pow(Math.abs(forward), DRIVE_CURVE_EXPONENT);
        if (strafe != 0) strafe = Math.signum(strafe) * Math.pow(Math.abs(strafe), DRIVE_CURVE_EXPONENT);
        if (rotate != 0) rotate = Math.signum(rotate) * Math.pow(Math.abs(rotate), DRIVE_CURVE_EXPONENT);
        strafe *= STRAFE_MULTIPLIER;
        rotate *= ROTATE_MULTIPLIER;

        // Mecanum formula
        double frontLeftPower = forward + rotate + strafe;
        double frontRightPower = forward - rotate - strafe;
        double backRightPower = forward - rotate + strafe;
        double backLeftPower = forward + rotate - strafe;

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

    // ======================== INTAKE ========================

    private void updateIntake() {
        boolean lt = gamepad1.left_trigger > 0.5 || gamepad2.left_trigger > 0.5;
        boolean dpadUp = gamepad1.dpad_up || gamepad2.dpad_up;
        boolean aNow = gamepad1.a || gamepad2.a;
        boolean bNow = gamepad1.b || gamepad2.b;
        boolean intakeReverse = lt && dpadUp;
        boolean intakeForward = lt && !dpadUp;

        // !! CW-ONLY intake — no CCW due to servo backlash !!
        // LT + A/B kept for future use but direction is always CW
        // Edge detection for A/B is updated at end of updateShootSequence (shared)

        if (intakeForward && !intakeRunning) {
            intakeRunning = true;
            intakeCycleStep = 0;
            spindexerPosIndex = 0;  // always start at S1 (CW-only)
            intakeCycleTimer.reset();
        }
        if (intakeReverse && !intakeRunning) {
            intakeRunning = true;
            intakeCycleStep = 0;
            spindexerPosIndex = 0;
            intakeCycleTimer.reset();
        }
        if (!intakeForward && !intakeReverse && intakeRunning) {
            intakeRunning = false;
            intakeCoastActive = true;
            intakeCoastTimer.reset();
            intakeCycleStep = 0;
            spindexerPosIndex = 0;  // CW-only: always reset to S1
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

    // ======================== INTAKE SPINDEXER ========================

    private void updateIntakeSpindexer() {
        if (spindexer == null) return;
        if (shootState != ShootState.IDLE && shootState != ShootState.DONE) return;

        if (intakeRunning || intakeCoastActive) {
            // Always cycle through all 3 positions (dumb cycle)
            // Smart sensor intake disabled — false positives cause sticking.
            // Re-enable after calibrating sensor thresholds with ColorSensorTest.
            if (intakeCycleTimer.milliseconds() >= SPINDEXER_INTAKE_CYCLE_MS) {
                intakeCycleTimer.reset();
                advanceIntakePosIndex();
            }
            spindexer.setPosition(SPINDEXER_SHOOT_ALL[spindexerPosIndex] + SPINDEXER_INTAKE_OFFSET);
            return;
        }

        // Idle: hold current manual slot
        spindexer.setPosition(getSpindexerShootPos(manualSpindexerSlot));
    }

    // ======================== SHOOT SEQUENCE ========================

    private void updateShootSequence() {
        boolean rtHeld = gamepad1.right_trigger > 0.5 || gamepad2.right_trigger > 0.5;
        boolean rbHeld = gamepad1.right_bumper || gamepad2.right_bumper;
        boolean aHeld = gamepad1.a || gamepad2.a;
        boolean bHeld = gamepad1.b || gamepad2.b;
        boolean aRising = aHeld && !prevA;
        boolean bRising = bHeld && !prevB;

        // --- RT pressed: Auto shoot (vision-assisted) ---
        if (rtHeld && !prevShootRT && shootState == ShootState.IDLE) {
            // Determine color filter
            if (aHeld) shootColorMode = ShootColorMode.PURPLE_ONLY;
            else if (bHeld) shootColorMode = ShootColorMode.GREEN_ONLY;
            else shootColorMode = ShootColorMode.ALL;

            // Pre-check: do we have balls matching the filter?
            boolean canStart = true;
            if (shootColorMode != ShootColorMode.ALL && hasSmartSensors()) {
                canStart = false;
                for (int i = 0; i < 3; i++) {
                    if ((shootColorMode == ShootColorMode.PURPLE_ONLY && slotBalls[i] == BallColor.PURPLE) ||
                            (shootColorMode == ShootColorMode.GREEN_ONLY && slotBalls[i] == BallColor.GREEN)) {
                        canStart = true; break;
                    }
                }
            }

            if (canStart) {
                shootPosIndex = 0;
                shootScanCount = 0;
                currentSpindexerSlot = SPINDEXER_SLOT_MAP[0];
                flywheelSpinning = true;
                stateTimer.reset();
                manualShootMode = false;
                if (hasVisionTarget()) {
                    shootState = ShootState.ALIGNING;
                } else {
                    shootState = ShootState.INDEXING;
                }
            }
        }

        // --- RB pressed: Manual shoot (fixed 1600 RPM, no vision) ---
        if (rbHeld && !prevShootRB && shootState == ShootState.IDLE) {
            shootPosIndex = 0;
            shootScanCount = 0;
            currentSpindexerSlot = SPINDEXER_SLOT_MAP[0];
            shootColorMode = ShootColorMode.ALL;  // manual always shoots all
            flywheelSpinning = true;
            stateTimer.reset();
            manualShootMode = true;
            shootState = ShootState.INDEXING;
        }

        // Release triggers -> stop shooting
        if (!rtHeld && !rbHeld && shootState != ShootState.IDLE && shootState != ShootState.DONE) {
            flywheelSpinning = false;
            manualSpindexerSlot = currentSpindexerSlot;
            shootState = ShootState.IDLE;
        }
        prevShootRT = rtHeld;
        prevShootRB = rbHeld;

        // LB = shooter off
        if (gamepad1.left_bumper || gamepad2.left_bumper) {
            flywheelSpinning = false;
            manualSpindexerSlot = currentSpindexerSlot;
            shootState = ShootState.IDLE;
        }

        // Kicker safety: only engages in KICKER_UP state
        if (servoArm != null) {
            if (intakeRunning || intakeCoastActive || shootState != ShootState.KICKER_UP) {
                servoArm.setPosition(KICKER_DOWN_POS);
            }
        }

        // Flywheel control
        if (shooterOne != null && shootertwo != null && flywheelSpinning) {
            if (manualShootMode) {
                // Fixed safe close-range velocity (3000 RPM)
                shooterOne.setVelocity(VELOCITY_DEFAULT);
                shootertwo.setVelocity(VELOCITY_DEFAULT);
                hoodPosition = MANUAL_SHOOT_HOOD;
            } else {
                // Auto mode: direct ty -> velocity
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
            // Update A/B edge detection here (shared with intake)
            prevA = aHeld;
            prevB = bHeld;
            return;
        }

        if (servoArm == null || spindexer == null) {
            prevA = aHeld;
            prevB = bHeld;
            return;
        }

        switch (shootState) {
            case ALIGNING:
                hoodPosition = computeHoodFromTy();
                updateHoodServos();
                if (isAligned() || stateTimer.seconds() > ALIGNMENT_TIMEOUT_SEC) {
                    shootState = ShootState.INDEXING;
                    stateTimer.reset();
                }
                break;

            case INDEXING:
                spindexer.setPosition(SPINDEXER_SHOOT_ALL[shootPosIndex]);
                if (manualShootMode && stateTimer.seconds() < MANUAL_SHOOT_SPINUP_SEC) break;
                if (!isKickerSafeToRun()) break;
                int indexMs = manualShootMode ? MANUAL_INDEX_MS : SPINDEXER_INDEX_MS;
                if (stateTimer.milliseconds() >= indexMs) {
                    // Check color filter before kicking
                    int slot = SPINDEXER_SLOT_MAP[shootPosIndex];
                    boolean colorOk;
                    if (!hasSmartSensors()) {
                        colorOk = true; // no sensors -> shoot everything
                    } else {
                        switch (shootColorMode) {
                            case PURPLE_ONLY: colorOk = slotBalls[slot] == BallColor.PURPLE; break;
                            case GREEN_ONLY:  colorOk = slotBalls[slot] == BallColor.GREEN; break;
                            default:          colorOk = true; break;
                        }
                    }
                    if (colorOk) {
                        shootScanCount = 0;
                        shootState = ShootState.KICKER_UP;
                        stateTimer.reset();
                    } else {
                        // Color doesn't match -> skip this slot, advance
                        advanceShootPosIndex();
                        shootScanCount++;
                        if (shootScanCount >= SPINDEXER_SHOOT_ALL.length) {
                            // All positions scanned, no match -> stop
                            flywheelSpinning = false;
                            shootState = ShootState.IDLE;
                        } else {
                            stateTimer.reset();
                        }
                    }
                }
                break;

            case KICKER_UP:
                spindexer.setPosition(SPINDEXER_SHOOT_ALL[shootPosIndex]);
                servoArm.setPosition(KICKER_UP_POS);
                int kickUpMs = manualShootMode ? MANUAL_KICKER_UP_MS : KICKER_UP_MS;
                if (stateTimer.milliseconds() >= kickUpMs) {
                    shootState = ShootState.KICKER_DOWN;
                    stateTimer.reset();
                }
                break;

            case KICKER_DOWN:
                servoArm.setPosition(KICKER_DOWN_POS);
                spindexer.setPosition(SPINDEXER_SHOOT_ALL[shootPosIndex]);
                int kickDownMs = manualShootMode ? MANUAL_KICKER_DOWN_MS : KICKER_DOWN_MS;
                if (stateTimer.milliseconds() >= kickDownMs) {
                    // Advance to next position (forward only, wrap at hardware limit)
                    advanceShootPosIndex();
                    shootScanCount = 0;
                    shootState = ShootState.INDEXING;
                    stateTimer.reset();
                }
                break;

            default:
                break;
        }

        // Update A/B edge detection (shared with intake)
        prevA = aHeld;
        prevB = bHeld;
    }

    private boolean isKickerSafeToRun() {
        return shootState != ShootState.KICKER_UP && shootState != ShootState.KICKER_DOWN;
    }

    // ======================== SPINDEXER HELPERS ========================

    /** Spindexer position for a specific logical slot (Rotation 1 only). Used for idle hold. */
    private double getSpindexerShootPos(int slot) {
        if (slot == 0) return SPINDEXER_SHOOT_0;
        else if (slot == 1) return SPINDEXER_SHOOT_1;
        else return SPINDEXER_SHOOT_2;
    }

    /**
     * Advance intake position index.
     * !! CW-ONLY !! After S2 (index 2), rewind to S1 (index 0). Never go CCW — backlash.
     */
    private void advanceIntakePosIndex() {
        spindexerPosIndex++;
        if (spindexerPosIndex >= SPINDEXER_SHOOT_ALL.length) {
            spindexerPosIndex = 0;  // rewind to S1 after S2
        }
        intakeCycleStep = SPINDEXER_SLOT_MAP[spindexerPosIndex];
    }

    /**
     * Advance shoot position index.
     * !! CW-ONLY !! After S2 (index 2), rewind to S1 (index 0). Never go CCW — backlash.
     */
    private void advanceShootPosIndex() {
        shootPosIndex++;
        if (shootPosIndex >= SPINDEXER_SHOOT_ALL.length) {
            shootPosIndex = 0;  // rewind to S1 after S2
        }
        currentSpindexerSlot = SPINDEXER_SLOT_MAP[shootPosIndex];
    }

    // ======================== SMART SPINDEXER (Color Sensors) ========================

    /** True if all 3 color sensors are configured. */
    private boolean hasSmartSensors() {
        return cs2 != null && cs3 != null && colorSenor != null;
    }

    /** Detect ball color at a specific sensor index. 0=cs3, 1=colorSenor, 2=cs2. */
    private BallColor detectBallColor(int sensorIndex) {
        ColorSensor sensor;
        if (sensorIndex == 0) sensor = cs3;
        else if (sensorIndex == 1) sensor = colorSenor;
        else sensor = cs2;
        if (sensor == null) return BallColor.EMPTY;
        int r = sensor.red(), g = sensor.green(), b = sensor.blue();
        int total = r + g + b;
        if (total < BALL_MIN_BRIGHTNESS) return BallColor.EMPTY;
        if (g > r + BALL_GREEN_DOMINANCE && g > b + BALL_GREEN_DOMINANCE) return BallColor.GREEN;
        if (r > BALL_PURPLE_MIN && b > BALL_PURPLE_MIN && (r + b) > g + BALL_GREEN_DOMINANCE) return BallColor.PURPLE;
        return BallColor.EMPTY; // bright but indeterminate color
    }

    /** Read raw sensor by index: 0=cs3, 1=colorSenor, 2=cs2. Returns true if ball detected. */
    private boolean sensorDetectsBall(int sensorIndex) {
        return detectBallColor(sensorIndex) != BallColor.EMPTY;
    }

    /** Update slotBalls[] from all 3 sensors, accounting for turntable rotation. */
    private void updateSlotColors() {
        if (!hasSmartSensors()) return;
        int activeSlot = manualSpindexerSlot;
        if (intakeRunning || intakeCoastActive) activeSlot = intakeCycleStep;
        else if (shootState != ShootState.IDLE && shootState != ShootState.DONE) activeSlot = currentSpindexerSlot;

        for (int slot = 0; slot < 3; slot++) {
            int sensorIndex;
            if (SENSOR_ROTATION_SIGN > 0) {
                sensorIndex = ((slot - activeSlot) % 3 + 3) % 3;
            } else {
                sensorIndex = (slot + activeSlot) % 3;
            }
            slotBalls[slot] = detectBallColor(sensorIndex);
        }
    }

    /**
     * Check if a logical slot has a ball, accounting for turntable rotation.
     * @param slot       logical slot to check (0, 1, or 2)
     * @param activeSlot the slot currently at the intake/kicker position
     */
    private boolean isSlotOccupied(int slot, int activeSlot) {
        if (!hasSmartSensors()) return false;
        int sensorIndex;
        if (SENSOR_ROTATION_SIGN > 0) {
            sensorIndex = ((slot - activeSlot) % 3 + 3) % 3;
        } else {
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

    // ======================== VELOCITY / HOOD (direct ty-based) ========================

    /** Compute target velocity directly from Limelight ty (no intermediate distance). */
    private double computeTargetVelocity() {
        if (!hasVisionTarget()) return targetVelocity;
        double ty = getVisionTyDegrees();
        if (ty <= FAR_TY_THRESHOLD) return VELOCITY_FAR_SIDE_MAX;
        double vel = interpolateFromAnchors(ty, SHOOTER_ANCHOR_TY, SHOOTER_TY_VELOCITY);
        if (vel < VELOCITY_MIN) vel = VELOCITY_MIN;
        if (vel > VELOCITY_MAX) vel = VELOCITY_MAX;
        return vel;
    }

    /** Compute hood position directly from Limelight ty (no intermediate distance). */
    private double computeHoodFromTy() {
        if (!hasVisionTarget()) return hoodPosition;
        double ty = getVisionTyDegrees();
        double h = interpolateFromAnchors(ty, SHOOTER_ANCHOR_TY, SHOOTER_TY_HOOD);
        if (h < 0) h = 0;
        if (h > 1) h = 1;
        return h;
    }

    /**
     * Map internal hoodPosition (0.0=low, 1.0=high) to 5-turn servo position.
     * Inverted: hoodPosition 0.0 -> servo 0.500 (bottom), hoodPosition 1.0 -> servo 0.180 (top).
     */
    private double mapHoodToServoPosition(double hoodPos) {
        hoodPos = Math.max(0.0, Math.min(1.0, hoodPos));
        return HOOD_POS_BOTTOM - (hoodPos * (HOOD_POS_BOTTOM - HOOD_POS_TOP));
    }

    /** Linear interpolation from anchor arrays. xVal = input; xAnchors/yAnchors = same length. */
    private double interpolateFromAnchors(double xVal, double[] xAnchors, double[] yAnchors) {
        if (xAnchors == null || yAnchors == null || xAnchors.length != yAnchors.length || xAnchors.length == 0)
            return yAnchors != null && yAnchors.length > 0 ? yAnchors[0] : 0;
        if (xVal <= xAnchors[0]) return yAnchors[0];
        if (xVal >= xAnchors[xAnchors.length - 1]) return yAnchors[yAnchors.length - 1];
        for (int i = 0; i < xAnchors.length - 1; i++) {
            if (xVal >= xAnchors[i] && xVal <= xAnchors[i + 1]) {
                double t = (xVal - xAnchors[i]) / (xAnchors[i + 1] - xAnchors[i]);
                return yAnchors[i] + t * (yAnchors[i + 1] - yAnchors[i]);
            }
        }
        return yAnchors[yAnchors.length - 1];
    }

    private boolean isAligned() {
        if (!hasVisionTarget()) return false;
        double tx = getVisionTxDegrees();
        double txErr = (Math.abs(tx) < 999) ? (tx - ALIGNMENT_TX_OFFSET_DEG) : 999;
        return Math.abs(txErr) <= ALIGNMENT_TOLERANCE_DEG;
    }

    private double wrapDeg(double d) {
        while (d > 180) d -= 360;
        while (d < -180) d += 360;
        return d;
    }

    // ======================== HOOD ========================

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
        if (inShootSequence && !manualShootMode) {
            hoodPosition = computeHoodFromTy();
        }
        // (manual preset hood is set in updateShootSequence)
        double servoPos = mapHoodToServoPosition(hoodPosition);
        hoodOne.setPosition(servoPos);
        hood.setPosition(HOOD_POS_TOP + HOOD_POS_BOTTOM - servoPos);
    }

    // ======================== BALL DETECTION (primary sensor) ========================

    private boolean isGreenBallDetected() {
        if (colorSenor == null) return false;
        int r = colorSenor.red(), g = colorSenor.green(), b = colorSenor.blue();
        int total = r + g + b;
        if (total < BALL_MIN_BRIGHTNESS) return false;
        return g > r + BALL_GREEN_DOMINANCE && g > b + BALL_GREEN_DOMINANCE;
    }

    private boolean isPurpleBallDetected() {
        if (colorSenor == null) return false;
        int r = colorSenor.red(), g = colorSenor.green(), b = colorSenor.blue();
        int total = r + g + b;
        if (total < BALL_MIN_BRIGHTNESS) return false;
        return r > BALL_PURPLE_MIN && b > BALL_PURPLE_MIN && (r + b) > g + BALL_GREEN_DOMINANCE;
    }

    // ======================== LED ========================

    private void updateLedFromColor() {
        if (led == null) return;

        // --- 3-ball flash: flash red 3 times quickly when all slots full ---
        boolean allFull = hasSmartSensors()
                && slotBalls[0] != BallColor.EMPTY
                && slotBalls[1] != BallColor.EMPTY
                && slotBalls[2] != BallColor.EMPTY;
        if (allFull && !prevAllSlotsFull) {
            ledFlashActive = true;
            ledFlashTimer.reset();
        }
        prevAllSlotsFull = allFull;

        if (ledFlashActive) {
            int phase = (int)(ledFlashTimer.milliseconds() / 100);  // 100ms per phase
            if (phase >= 6) {
                // 3 on/off cycles complete (on-off-on-off-on-off = 6 phases)
                ledFlashActive = false;
            } else {
                led.setPosition(phase % 2 == 0 ? LED_POS_RED : LED_POS_OFF);
                return;  // override all other LED logic during flash
            }
        }

        // --- Normal LED logic ---
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
            led.setPosition(LED_POS_YELLOW);
            return;
        }

        boolean goalPostDetected = hasVisionTarget() || hasAnyGoalTarget();
        boolean greenBall = isGreenBallDetected();
        boolean purpleBall = isPurpleBallDetected();

        if (goalPostDetected) {
            boolean blinkOn = ((int) (ledBlinkTimer.milliseconds() / LED_BLINK_MS)) % 2 == 0;
            double goalColor = greenBall ? LED_POS_GREEN_BALL : (purpleBall ? LED_POS_PURPLE_BALL : LED_POS_YELLOW);
            led.setPosition(blinkOn ? goalColor : LED_POS_OFF);
            return;
        }
        if (greenBall) { led.setPosition(LED_POS_GREEN_BALL); return; }
        if (purpleBall) { led.setPosition(LED_POS_PURPLE_BALL); return; }
        led.setPosition(LED_POS_RED);
    }

    // ======================== TELEMETRY ========================

    private void updateTelemetry() {
        telemetry.addData("Status", "Running");
        telemetry.addData("Alliance", "BLUE");
        try { telemetry.addData("Battery", String.format("%.2f V", hardwareMap.voltageSensor.iterator().next().getVoltage())); } catch (Exception e) {}
        telemetry.addData("Shoot", shootState.toString());
        String modeStr = manualShootMode
                ? "MANUAL (1600 RPM safe)"
                : "AUTO (vision, " + shootColorMode.toString() + ")";
        telemetry.addData("Mode", modeStr);
        telemetry.addData("Flywheel", flywheelSpinning ? "ON" : "OFF");

        // Actual vs target flywheel velocity (tks/s) and converted motor RPM
        if (shooterOne != null) {
            double targetTks = computeTargetVelocity();
            double actualTks = shooterOne.getVelocity();
            double targetRPM = (targetTks / MOTOR_ENCODER_CPR) * 60.0;
            double actualRPM = (actualTks / MOTOR_ENCODER_CPR) * 60.0;
            double pctMax = (actualTks / 2800.0) * 100.0;  // % of motor max @ 12V
            telemetry.addData("Flywheel Target", String.format("%d tks/s  (%d RPM)", (int) targetTks, (int) targetRPM));
            telemetry.addData("Flywheel Actual", String.format("%d tks/s  (%d RPM)  [%.0f%% max]", (int) actualTks, (int) actualRPM, pctMax));
        }

        telemetry.addData("Intake dir", "CW only (1>0>2) — backlash-safe");
        telemetry.addData("Shoot pos idx", shootPosIndex + "/" + (SPINDEXER_SHOOT_ALL.length - 1));
        telemetry.addData("Intake pos idx", spindexerPosIndex + "/" + (SPINDEXER_SHOOT_ALL.length - 1));

        // Slot ball colors from smart sensors
        if (hasSmartSensors()) {
            int activeSlot = manualSpindexerSlot;
            if (intakeRunning || intakeCoastActive) activeSlot = intakeCycleStep;
            else if (shootState != ShootState.IDLE && shootState != ShootState.DONE) activeSlot = currentSpindexerSlot;
            telemetry.addData("Slot 0", slotBalls[0].toString());
            telemetry.addData("Slot 1", slotBalls[1].toString());
            telemetry.addData("Slot 2", slotBalls[2].toString());
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

        // Limelight / Goal post
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
            double ty = getVisionTyDegrees();
            double txErr = (Math.abs(tx) < 999) ? (tx - ALIGNMENT_TX_OFFSET_DEG) : 0;
            double velFromTy = computeTargetVelocity();
            double velRPM = (velFromTy / MOTOR_ENCODER_CPR) * 60.0;
            telemetry.addData("Vision (shoot)", limelightEnabled ? "LimeLight" : "AprilTag");
            telemetry.addData("Alliance target", "YES");
            telemetry.addData("Distance (LL)", String.format("%.1f in  (real ≈%.1f in)", dist, dist * 0.917));
            telemetry.addData("Raw ty", String.format("%.1f deg", ty));
            telemetry.addData("Computed vel (ty)", String.format("%.0f tks/s  (%d RPM)", velFromTy, (int) velRPM));
            telemetry.addData("Computed hood (ty)", String.format("%.3f", computeHoodFromTy()));
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
                else if (!anyGoal) telemetry.addLine("LimeLight: no goal tag in view");
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
        } else {
            telemetry.addData("Color", "No sensor (REV config: colorSenor or colorSensor)");
        }

        if (led != null) {
            if (ledFlashActive) telemetry.addData("LED", "FLASH RED (3 balls loaded!)");
            else if (hasVisionTarget() || hasAnyGoalTarget()) telemetry.addData("LED", "BLINK (goal)");
            else if (isGreenBallDetected()) telemetry.addData("LED", "GREEN (ball)");
            else if (isPurpleBallDetected()) telemetry.addData("LED", "PURPLE (ball)");
            else telemetry.addData("LED", "OFF (no ball)");
        }

        telemetry.addData("Shoot (RT)", (gamepad1.right_trigger > 0.5 || gamepad2.right_trigger > 0.5) ? "HELD" : "---");
        telemetry.addData("Manual (RB)", (gamepad1.right_bumper || gamepad2.right_bumper) ? "HELD" : "---");

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

    // ======================== STOP ========================

    private void stopAll() {
        if (leftFront != null) leftFront.setPower(0);
        if (rightFront != null) rightFront.setPower(0);
        if (leftBack != null) leftBack.setPower(0);
        if (rightBack != null) rightBack.setPower(0);
        if (intake != null) intake.setPower(0);
        if (shooterOne != null) shooterOne.setPower(0);
        if (shootertwo != null) shootertwo.setPower(0);
        if (servoArm != null) servoArm.setPosition(KICKER_DOWN_POS);
        if (spindexer != null) spindexer.setPosition(SPINDEXER_SHOOT_1);
    }
}

