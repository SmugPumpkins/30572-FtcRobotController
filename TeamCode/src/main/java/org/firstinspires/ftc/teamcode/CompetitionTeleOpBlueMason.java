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
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

/**
 * COMPETITION TELEOP BLUE MASON - Full shooter system. BLUE ALLIANCE ONLY.
 * Renamed and updated with GOLD STANDARD Odometry distance calculation.
 */

@TeleOp(name = "Competition TeleOp BLUE MASON", group = "Driver")
public class CompetitionTeleOpBlueMason extends LinearOpMode {

    // === DRIVE (18" x 18" FTC-legal robot - fast top speed, driver-friendly curve)
    // ===
    public static double MAX_SPEED = 1.0; // full speed for big robot
    public static double PRECISION_SPEED = 0.5; // D-pad Down held = slower for alignment/confidence
    public static double DRIVE_CURVE_EXPONENT = 2.0; // squared curve: small stick = fine control, full stick = full
    // power
    public static double STRAFE_MULTIPLIER = 0.9;
    public static double ROTATE_MULTIPLIER = 1.0;
    public static double DEAD_ZONE = 0.08; // slightly more responsive
    public static double INTAKE_POWER = 0.8;

    // === GOLD STANDARD: GOAL POSITION (Odometry) ===
    /**
     * X,Y coordinates of the Blue Alliance Goal. Adjust during field calibration.
     */
    public static double BLUE_GOAL_X = 0.0;
    public static double BLUE_GOAL_Y = 72.0;

    // === SHOOTER - distance-based from FTC positions (anchor table +
    // interpolation) ===
    public static double VELOCITY_MIN = 800;
    public static double VELOCITY_MAX = 2400;
    /**
     * Anchor: corrected distance (in) -> velocity (ticks/s). Calibrated: 68 in ->
     * 1350 RPM.
     */
    public static double[] SHOOTER_ANCHOR_DIST_IN = { 48, 68, 90 };
    public static double[] SHOOTER_ANCHOR_VELOCITY = { 1100, 1350, 1700 };
    /**
     * Anchor: corrected distance (in) -> hood position [0,1]. Lower hood at close =
     * ty ~0.
     */
    public static double[] SHOOTER_ANCHOR_HOOD = { 0.55, 0.72, 0.95 };
    /**
     * Backspin adds lift: reduce RPM or lower hood. 1.0 = no change; try 0.95 if
     * shots go high.
     */
    public static double BACKSPIN_RPM_FACTOR = 1.0;
    /**
     * Backspin: add to hood (negative = lower hood). Try -0.03 if shots go high.
     */
    public static double BACKSPIN_HOOD_OFFSET = 0.0;
    public static double VELOCITY_DEFAULT = 1350; // fallback when no vision
    /** Far-side: use max power (RPM). Vision distance >= this = far shot. */
    public static double FAR_DISTANCE_THRESHOLD_IN = 120.0; // inches
    public static double VELOCITY_FAR_SIDE_MAX = 6000.0; // RPM for far shots

    // === HOOD (2 servos, distance -> position) ===
    public static double HOOD_POS_NEAR = 0;
    public static double HOOD_POS_FAR = 1.0;
    public static double HOOD_DIST_NEAR = 24;
    public static double HOOD_DIST_FAR = 72;

    // === SPINDEXER: two sets (shooting = kick plate, intake = indexer ramp) ===
    public static double SPINDEXER_SHOOT_0 = 0.150;
    public static double SPINDEXER_SHOOT_1 = 0.550;
    public static double SPINDEXER_SHOOT_2 = 0.1000;
    public static double SPINDEXER_INTAKE_0 = 0.110;
    public static double SPINDEXER_INTAKE_1 = 0.510;
    public static double SPINDEXER_INTAKE_2 = 0.910;
    public static int SPINDEXER_INDEX_MS = 500;
    public static int SPINDEXER_INTAKE_CYCLE_MS = 450;
    public static double INTAKE_COAST_SEC = 3.0;

    // === KICKER (servoArm: up = ball out, down = ready for next) ===
    public static double KICKER_UP_POS = 0.060;
    public static double KICKER_DOWN_POS = 0.330;
    public static int KICKER_UP_MS = 220;
    public static int KICKER_DOWN_MS = 80;

    // === ALIGNMENT ===
    public static double ALIGNMENT_TOLERANCE_DEG = 2.0;
    public static double ALIGNMENT_TX_OFFSET_DEG = 0.0;
    public static double ALIGNMENT_CORRECTION_SIGN = -1.0;
    public static double ALIGNMENT_STRAFE_GAIN = 0.03;
    public static double ALIGNMENT_ROTATE_GAIN = 0.04;
    public static double AUTO_DRIVE_ROTATE_GAIN = 0.02;
    public static double AUTO_DRIVE_FORWARD_GAIN = 0.015;
    public static double INTAKE_ORIENTATION_TRIGGER_THRESHOLD = 0.2;

    // === DEFENSIVE DRIVE (Pinpoint position hold during shooting - resist being
    // pushed) ===
    public static boolean DEFENSIVE_LOCK_ENABLED = true;
    public static double DEFENSIVE_LOCK_KP_X = 0.03;
    public static double DEFENSIVE_LOCK_KP_Y = 0.03;
    public static double DEFENSIVE_LOCK_KP_HEADING = 0.02;
    public static double DEFENSIVE_LOCK_DEADBAND_IN = 0.5;
    public static double DEFENSIVE_LOCK_DEADBAND_DEG = 2.0;
    public static double DEFENSIVE_LOCK_MAX_CORRECTION = 0.4;

    // === MANUAL SHOOT (override when no AprilTag) - FASTEST THROUGHPUT ===
    public static double MANUAL_SHOOT_POWER = 0.8;
    public static double MANUAL_SHOOT_SPINUP_SEC = 1.0;
    public static int MANUAL_INDEX_MS = 500;
    public static int MANUAL_KICKER_UP_MS = 200;
    public static int MANUAL_KICKER_DOWN_MS = 70;

    // === LED ===
    public static double LED_POS_OFF = 0.0;
    public static double LED_POS_RED = 0.35;
    public static double LED_POS_YELLOW = 0.388;
    public static double LED_POS_GREEN_BALL = 0.500;
    public static double LED_POS_PURPLE_BALL = 0.666;
    public static double LED_POS_WHITE = 0.722;
    public static int LED_BLINK_MS = 250;
    public static int BALL_MIN_BRIGHTNESS = 50;
    public static int BALL_GREEN_DOMINANCE = 5;
    public static int BALL_PURPLE_MIN = 30;

    // === GOAL POST TAGS ===
    public static int[] BLUE_TAG_IDS = { 20 };
    public static int[] RED_TAG_IDS = { 24 };
    public static double DISTANCE_CORRECTION = 0.95;

    // === DRIVE ===
    private DcMotor leftFront, rightFront, leftBack, rightBack;
    private DcMotor intake;
    private IMU imu;

    // === SHOOTER ===
    private DcMotorEx shooterOne, shootertwo;

    // === HOOD ===
    private Servo hoodOne, hood;

    // === SPINDEXER + KICKER ===
    private Servo spindexer, servoArm;

    // === SENSORS ===
    private ColorSensor colorSenor;
    private Servo led;

    // === VISION ===
    private LimeLightMason vision;
    private Limelight3A limelight;
    private boolean limelightEnabled = false;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private boolean visionEnabled = false;

    // === PINPOINT ODOMETRY ===
    private GoBildaPinpointDriver pinpoint = null;
    private boolean pinpointEnabled = false;

    // === STATE ===
    private double targetVelocity = VELOCITY_DEFAULT;
    private int velocityPreset = 1;
    private boolean intakeRunning = false;
    private boolean intakeCycleToggle = false;
    private double hoodPosition = 0.5;
    private boolean flywheelSpinning = false;
    private boolean prevG1LB, prevG2RB, prevG2LB, prevG2A, prevB;
    private int manualSpindexerSlot = 0;
    private boolean intakeCoastActive = false;
    private ElapsedTime intakeCoastTimer = new ElapsedTime();
    private int intakeCycleStep = 0;
    private ElapsedTime intakeCycleTimer = new ElapsedTime();
    private ElapsedTime ledBlinkTimer = new ElapsedTime();
    private boolean driveOrientationIntakeFront = false;

    private enum ShootState {
        IDLE, ALIGNING, INDEXING, KICKER_UP, KICKER_DOWN, DONE
    }

    private ShootState shootState = ShootState.IDLE;
    private boolean manualShootMode = false;
    private int currentSpindexerSlot = 0;
    private ElapsedTime stateTimer = new ElapsedTime();
    private boolean prevShootBumper = false;

    private boolean defensiveLockActive = false;
    private double lockTargetX = 0, lockTargetY = 0, lockTargetHeading = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        initHardware();
        stopAll();
        telemetry.addData("Status", "Ready! Right Bumper = One-Button Shoot");
        telemetry.addData("Code Version", 12); // VERSION 12
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (pinpoint != null)
                pinpoint.update();
            updateDrive();
            updateIntake();
            updateIntakeSpindexer();
            updateShootSequence();
            updateHood();
            updateLedFromColor();
            updateTelemetry();
        }
        stopAll();
        if (limelight != null && limelightEnabled)
            limelight.stop();
        if (visionPortal != null)
            visionPortal.close();
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

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
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

        if (spindexer instanceof ServoImplEx)
            ((ServoImplEx) spindexer).setPwmRange(new PwmControl.PwmRange(500, 2500));
        if (servoArm instanceof ServoImplEx)
            ((ServoImplEx) servoArm).setPwmRange(new PwmControl.PwmRange(500, 2500));
        if (hoodOne instanceof ServoImplEx)
            ((ServoImplEx) hoodOne).setPwmRange(new PwmControl.PwmRange(500, 2500));
        if (hood != null && hood instanceof ServoImplEx)
            ((ServoImplEx) hood).setPwmRange(new PwmControl.PwmRange(500, 2500));

        try {
            colorSenor = hardwareMap.get(ColorSensor.class, "colorSenor");
        } catch (Exception e) {
            colorSenor = null;
        }
        if (colorSenor == null)
            try {
                colorSenor = hardwareMap.get(ColorSensor.class, "colorSensor");
            } catch (Exception e2) {
            }
        if (colorSenor != null) {
            try {
                colorSenor.getClass().getMethod("enableLed", boolean.class).invoke(colorSenor, true);
            } catch (Exception e) {
            }
        }
        try {
            led = hardwareMap.get(Servo.class, "led");
        } catch (Exception e) {
            led = null;
        }
        if (led != null && led instanceof ServoImplEx)
            ((ServoImplEx) led).setPwmRange(new PwmControl.PwmRange(500, 2500));

        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD);
            pinpoint.setOffsets(-2.5, 1, DistanceUnit.INCH);
            pinpoint.resetPosAndIMU();
            pinpointEnabled = true;
        } catch (Exception e) {
            pinpoint = null;
            pinpointEnabled = false;
        }

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(0);
            limelight.start();
            limelightEnabled = true;
        } catch (Exception e) {
            limelight = null;
            limelightEnabled = false;
        }

        try {
            aprilTag = new AprilTagProcessor.Builder().setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)
                    .build();
            visionPortal = new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag).build();
            visionEnabled = true;
        } catch (Exception e) {
            visionEnabled = false;
        }

        vision = new LimeLightMason();
        vision.initLimelight(limelight);
        vision.initAprilTagProcessor(visionPortal, aprilTag);
    }

    /**
     * GOLD STANDARD: Returns distance from robot to Blue Goal based on Pinpoint
     * coordinates.
     */
    private double getOdometryDistanceInches() {
        if (pinpoint == null)
            return 48;
        double currentX = pinpoint.getPosX(DistanceUnit.INCH);
        double currentY = pinpoint.getPosY(DistanceUnit.INCH);
        double dx = BLUE_GOAL_X - currentX;
        double dy = BLUE_GOAL_Y - currentY;
        return Math.sqrt(dx * dx + dy * dy);
    }

    private double getVisionDistanceInches() {
        // GOLD STANDARD: Favor Odometry for shooter logic
        if (pinpointEnabled && pinpoint != null) {
            return getOdometryDistanceInches();
        }
        double raw = vision.getVisionDistanceInchesRaw();
        return raw * DISTANCE_CORRECTION;
    }

    private void updateDrive() {
        if (gamepad1.x || gamepad2.x)
            imu.resetYaw();

        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);
        theta = AngleUnit.normalizeRadians(theta - robotYaw);
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);
        forward = newForward;
        double strafe = newRight;

        if ((gamepad1.y || gamepad2.y) && vision.hasVisionTarget()) {
            double tx = vision.getVisionTxDegrees();
            double txErr = (Math.abs(tx) < 999) ? (tx - ALIGNMENT_TX_OFFSET_DEG) : 0;
            double dist = getVisionDistanceInches();
            if (Math.abs(txErr) > ALIGNMENT_TOLERANCE_DEG) {
                double corr = ALIGNMENT_CORRECTION_SIGN * txErr;
                rotate = corr * ALIGNMENT_ROTATE_GAIN;
                strafe += corr * ALIGNMENT_STRAFE_GAIN;
            }
            if (dist > 24 && dist < 999) {
                forward = (dist - 24) * AUTO_DRIVE_FORWARD_GAIN;
                if (forward > 0.5)
                    forward = 0.5;
            }
        }

        if (shootState == ShootState.ALIGNING && vision.hasVisionTarget()) {
            double tx = vision.getVisionTxDegrees();
            double txErr = (Math.abs(tx) < 999) ? (tx - ALIGNMENT_TX_OFFSET_DEG) : 0;
            if (Math.abs(txErr) > ALIGNMENT_TOLERANCE_DEG) {
                double corr = ALIGNMENT_CORRECTION_SIGN * txErr;
                strafe += corr * ALIGNMENT_STRAFE_GAIN;
                rotate += corr * ALIGNMENT_ROTATE_GAIN;
            }
        }

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
            if (Math.abs(errX) < DEFENSIVE_LOCK_DEADBAND_IN)
                errX = 0;
            if (Math.abs(errY) < DEFENSIVE_LOCK_DEADBAND_IN)
                errY = 0;
            if (Math.abs(errH) < DEFENSIVE_LOCK_DEADBAND_DEG)
                errH = 0;
            double defTheta = Math.toRadians(currH);
            double forwardErr = errX * Math.cos(defTheta) + errY * Math.sin(defTheta);
            double strafeErr = -errX * Math.sin(defTheta) + errY * Math.cos(defTheta);
            double correctiveForward = Math.max(-DEFENSIVE_LOCK_MAX_CORRECTION,
                    Math.min(DEFENSIVE_LOCK_MAX_CORRECTION, DEFENSIVE_LOCK_KP_X * forwardErr));
            double correctiveStrafe = Math.max(-DEFENSIVE_LOCK_MAX_CORRECTION,
                    Math.min(DEFENSIVE_LOCK_MAX_CORRECTION, DEFENSIVE_LOCK_KP_Y * strafeErr));
            double correctiveRotate = Math.max(-DEFENSIVE_LOCK_MAX_CORRECTION,
                    Math.min(DEFENSIVE_LOCK_MAX_CORRECTION, DEFENSIVE_LOCK_KP_HEADING * errH));
            forward += correctiveForward;
            strafe += correctiveStrafe;
            rotate += correctiveRotate;
        } else
            defensiveLockActive = false;

        if (Math.abs(forward) < DEAD_ZONE)
            forward = 0;
        if (Math.abs(strafe) < DEAD_ZONE)
            strafe = 0;
        if (Math.abs(rotate) < DEAD_ZONE)
            rotate = 0;

        if (forward != 0)
            forward = Math.signum(forward) * Math.pow(Math.abs(forward), DRIVE_CURVE_EXPONENT);
        if (strafe != 0)
            strafe = Math.signum(strafe) * Math.pow(Math.abs(strafe), DRIVE_CURVE_EXPONENT);
        if (rotate != 0)
            rotate = Math.signum(rotate) * Math.pow(Math.abs(rotate), DRIVE_CURVE_EXPONENT);
        strafe *= STRAFE_MULTIPLIER;
        rotate *= ROTATE_MULTIPLIER;

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

    private void updateIntake() {
        boolean bNow = gamepad1.b || gamepad2.b;
        if (bNow && !prevB)
            intakeCycleToggle = !intakeCycleToggle;
        prevB = bNow;

        boolean lt = gamepad1.left_trigger > 0.5 || gamepad2.left_trigger > 0.5;
        boolean dpadUp = gamepad1.dpad_up || gamepad2.dpad_up;
        boolean intakeReverse = lt && dpadUp;
        boolean intakeForward = (lt && !dpadUp) || intakeCycleToggle;

        if (intakeForward && !intakeRunning) {
            intakeRunning = true;
            intakeCycleStep = 0;
            intakeCycleTimer.reset();
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
        if (intakeCoastActive && intakeCoastTimer.seconds() >= INTAKE_COAST_SEC)
            intakeCoastActive = false;

        if (intakeRunning || intakeCoastActive)
            intake.setPower(intakeReverse ? -INTAKE_POWER : INTAKE_POWER);
        else
            intake.setPower(0);
    }

    private void updateIntakeSpindexer() {
        if (spindexer == null)
            return;
        if (shootState != ShootState.IDLE && shootState != ShootState.DONE)
            return;

        if (intakeRunning || intakeCoastActive) {
            if (intakeCycleTimer.milliseconds() >= SPINDEXER_INTAKE_CYCLE_MS) {
                intakeCycleStep = (intakeCycleStep + 1) % 6;
                intakeCycleTimer.reset();
            }
            int slot = (intakeCycleStep <= 2) ? intakeCycleStep : (5 - intakeCycleStep);
            spindexer.setPosition(getSpindexerIntakePos(slot));
            return;
        }
        spindexer.setPosition(getSpindexerShootPos(manualSpindexerSlot));
    }

    private void updateShootSequence() {
        boolean shootBumper = gamepad1.right_bumper || gamepad2.right_bumper || gamepad1.right_trigger > 0.5
                || gamepad2.right_trigger > 0.5;
        if (shootBumper && !prevShootBumper && shootState == ShootState.IDLE) {
            flywheelSpinning = true;
            stateTimer.reset();
            // GOLD STANDARD: If pinning is enabled, we always have a "target"
            if (pinpointEnabled || vision.hasVisionTarget()) {
                manualShootMode = false;
                shootState = ShootState.ALIGNING;
            } else {
                manualShootMode = true;
                shootState = ShootState.INDEXING;
                currentSpindexerSlot = 0;
            }
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

        if (servoArm != null)
            if (intakeRunning || intakeCoastActive || shootState != ShootState.KICKER_UP)
                servoArm.setPosition(KICKER_DOWN_POS);

        if (shooterOne != null && shootertwo != null && flywheelSpinning) {
            if (manualShootMode) {
                shooterOne.setPower(MANUAL_SHOOT_POWER);
                shootertwo.setPower(MANUAL_SHOOT_POWER);
            } else {
                double vel = computeTargetVelocity();
                shooterOne.setVelocity(vel);
                shootertwo.setVelocity(vel);
            }
        } else if (shooterOne != null && shootertwo != null) {
            shooterOne.setPower(0);
            shootertwo.setPower(0);
        }

        if (shootState == ShootState.IDLE || shootState == ShootState.DONE) {
            if (servoArm != null)
                servoArm.setPosition(KICKER_DOWN_POS);
            return;
        }
        if (servoArm == null || spindexer == null)
            return;

        switch (shootState) {
            case ALIGNING:
                hoodPosition = computeHoodFromDistance();
                updateHoodServos();
                if (isAligned() || stateTimer.seconds() > 5) {
                    shootState = ShootState.INDEXING;
                    currentSpindexerSlot = 0;
                    stateTimer.reset();
                }
                break;
            case INDEXING:
                boolean bBtnIdx = gamepad1.b || gamepad2.b;
                if (bBtnIdx && !prevB) {
                    currentSpindexerSlot = (currentSpindexerSlot + 1) % 3;
                    stateTimer.reset();
                }
                prevB = bBtnIdx;
                spindexer.setPosition(getSpindexerShootPos(currentSpindexerSlot));
                if (manualShootMode && stateTimer.seconds() < MANUAL_SHOOT_SPINUP_SEC)
                    break;
                if (!isKickerSafeToRun())
                    break;
                int indexMs = manualShootMode ? MANUAL_INDEX_MS : SPINDEXER_INDEX_MS;
                if (stateTimer.milliseconds() >= indexMs) {
                    shootState = ShootState.KICKER_UP;
                    stateTimer.reset();
                }
                break;
            case KICKER_UP:
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
                    currentSpindexerSlot = (currentSpindexerSlot + 1) % 3;
                    shootState = ShootState.INDEXING;
                    stateTimer.reset();
                }
                break;
            default:
                break;
        }
        if (shootState != ShootState.IDLE && shootState != ShootState.DONE)
            prevB = gamepad1.b || gamepad2.b;
    }

    private boolean isKickerSafeToRun() {
        return shootState != ShootState.KICKER_UP && shootState != ShootState.KICKER_DOWN;
    }

    private double getSpindexerShootPos(int slot) {
        if (slot == 0)
            return SPINDEXER_SHOOT_0;
        if (slot == 1)
            return SPINDEXER_SHOOT_1;
        return SPINDEXER_SHOOT_2;
    }

    private double getSpindexerIntakePos(int slot) {
        if (slot == 0)
            return SPINDEXER_INTAKE_0;
        if (slot == 1)
            return SPINDEXER_INTAKE_1;
        return SPINDEXER_INTAKE_2;
    }

    private double computeTargetVelocity() {
        // GOLD STANDARD: If pinning is enabled, we always use odometry
        if (!pinpointEnabled && !vision.hasVisionTarget())
            return targetVelocity;
        double dist = getVisionDistanceInches();
        if (dist >= FAR_DISTANCE_THRESHOLD_IN)
            return VELOCITY_FAR_SIDE_MAX * BACKSPIN_RPM_FACTOR;
        double vel = interpolateFromAnchors(dist, SHOOTER_ANCHOR_DIST_IN, SHOOTER_ANCHOR_VELOCITY);
        vel *= BACKSPIN_RPM_FACTOR;
        if (vel < VELOCITY_MIN)
            vel = VELOCITY_MIN;
        if (vel > VELOCITY_MAX)
            vel = VELOCITY_MAX;
        return vel;
    }

    private double computeHoodFromDistance() {
        if (!pinpointEnabled && !vision.hasVisionTarget())
            return hoodPosition;
        double dist = getVisionDistanceInches();
        double h = interpolateFromAnchors(dist, SHOOTER_ANCHOR_DIST_IN, SHOOTER_ANCHOR_HOOD);
        h += BACKSPIN_HOOD_OFFSET;
        if (h < 0)
            h = 0;
        if (h > 1)
            h = 1;
        return h;
    }

    private double interpolateFromAnchors(double distIn, double[] distAnchors, double[] valueAnchors) {
        if (distAnchors == null || valueAnchors == null || distAnchors.length != valueAnchors.length
                || distAnchors.length == 0)
            return valueAnchors != null && valueAnchors.length > 0 ? valueAnchors[0] : 0;
        if (distIn <= distAnchors[0])
            return valueAnchors[0];
        if (distIn >= distAnchors[distAnchors.length - 1])
            return valueAnchors[valueAnchors.length - 1];
        for (int i = 0; i < distAnchors.length - 1; i++) {
            if (distIn >= distAnchors[i] && distIn <= distAnchors[i + 1]) {
                double t = (distIn - distAnchors[i]) / (distAnchors[i + 1] - distAnchors[i]);
                return valueAnchors[i] + t * (valueAnchors[i + 1] - valueAnchors[i]);
            }
        }
        return valueAnchors[valueAnchors.length - 1];
    }

    private boolean isAligned() {
        if (!pinpointEnabled && !vision.hasVisionTarget())
            return false;
        double tx = vision.getVisionTxDegrees();
        double txErr = (Math.abs(tx) < 999) ? (tx - ALIGNMENT_TX_OFFSET_DEG) : 999;
        return Math.abs(txErr) <= ALIGNMENT_TOLERANCE_DEG;
    }

    private double wrapDeg(double d) {
        while (d > 180)
            d -= 360;
        while (d < -180)
            d += 360;
        return d;
    }

    private void updateHood() {
        if (gamepad1.dpad_left || gamepad2.dpad_left)
            hoodPosition = Math.max(0, hoodPosition - 0.02);
        if (gamepad1.dpad_right || gamepad2.dpad_right)
            hoodPosition = Math.min(1, hoodPosition + 0.02);
        if (shootState != ShootState.ALIGNING)
            updateHoodServos();
    }

    private void updateHoodServos() {
        if (hoodOne == null || hood == null)
            return;
        boolean inShootSequence = shootState == ShootState.ALIGNING || shootState == ShootState.INDEXING
                || shootState == ShootState.KICKER_UP || shootState == ShootState.KICKER_DOWN;
        if (inShootSequence)
            hoodPosition = computeHoodFromDistance();
        hoodOne.setPosition(hoodPosition);
        hood.setPosition(1 - hoodPosition);
    }

    private boolean isGreenBallDetected() {
        if (colorSenor == null)
            return false;
        int r = colorSenor.red(), g = colorSenor.green(), b = colorSenor.blue();
        int total = r + g + b;
        if (total < BALL_MIN_BRIGHTNESS)
            return false;
        return g > r + BALL_GREEN_DOMINANCE && g > b + BALL_GREEN_DOMINANCE;
    }

    private boolean isPurpleBallDetected() {
        if (colorSenor == null)
            return false;
        int r = colorSenor.red(), g = colorSenor.green(), b = colorSenor.blue();
        int total = r + g + b;
        if (total < BALL_MIN_BRIGHTNESS)
            return false;
        return r > BALL_PURPLE_MIN && b > BALL_PURPLE_MIN && (r + b) > g + BALL_GREEN_DOMINANCE;
    }

    private void updateLedFromColor() {
        if (led == null)
            return;
        boolean inShootSequence = shootState != ShootState.IDLE && shootState != ShootState.DONE;
        boolean aligning = shootState == ShootState.ALIGNING && (pinpointEnabled || vision.hasVisionTarget())
                && !isAligned();
        boolean alignedOrEmptying = (shootState == ShootState.ALIGNING && isAligned())
                || shootState == ShootState.INDEXING || shootState == ShootState.KICKER_UP
                || shootState == ShootState.KICKER_DOWN;
        if (inShootSequence && aligning) {
            boolean blinkOn = ((int) (ledBlinkTimer.milliseconds() / LED_BLINK_MS)) % 2 == 0;
            led.setPosition(blinkOn ? LED_POS_YELLOW : LED_POS_OFF);
            return;
        }
        if (inShootSequence && alignedOrEmptying) {
            led.setPosition(LED_POS_YELLOW);
            return;
        }
        boolean goalPostDetected = vision.hasVisionTarget() || vision.hasAnyGoalTarget();
        boolean greenBall = isGreenBallDetected();
        boolean purpleBall = isPurpleBallDetected();
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
        telemetry.addData("Code Version", 12);
        try {
            telemetry.addData("Battery",
                    String.format("%.2f V", hardwareMap.voltageSensor.iterator().next().getVoltage()));
        } catch (Exception e) {
        }
        telemetry.addData("Shoot", shootState.toString());
        telemetry.addData("Dist Mode",
                pinpointEnabled ? "GOLD (Odometry)" : (vision.hasVisionTarget() ? "AUTO (Vision)" : "MANUAL"));
        telemetry.addData("Flywheel", flywheelSpinning ? "ON" : "OFF");
        telemetry.addData("Target Vel", (int) computeTargetVelocity());

        if ((gamepad1.a || gamepad2.a) && !prevG2A) {
            velocityPreset = (velocityPreset + 1) % 3;
            targetVelocity = velocityPreset == 0 ? SHOOTER_ANCHOR_VELOCITY[0]
                    : velocityPreset == 1 ? SHOOTER_ANCHOR_VELOCITY[1] : SHOOTER_ANCHOR_VELOCITY[2];
        }
        prevG2A = gamepad1.a || gamepad2.a;

        boolean anyGoal = vision.hasAnyGoalTarget();
        int closestTagId = vision.getClosestGoalTagId();
        double closestDist = vision.getClosestGoalDistanceInches();
        double closestTx = vision.getClosestGoalTx();
        if (limelightEnabled && limelight != null) {
            LLResult llRes = limelight.getLatestResult();
            telemetry.addData("LimeLight", llRes != null && llRes.isValid() ? "CONNECTED" : "NO RESULT");
        }
        if (anyGoal && closestTagId >= 0) {
            String which = vision.isTagIdInAlliance(closestTagId) ? "alliance" : "other";
            telemetry.addData("Goal post", "DETECTED (Tag " + closestTagId + ", "
                    + String.format("%.1f in", closestDist) + ", " + which + ")");
        } else
            telemetry.addData("Goal post", "NOT DETECTED");

        double dist = getVisionDistanceInches();
        telemetry.addData("Distance", String.format("%.1f in", dist));

        if (pinpoint != null && pinpointEnabled) {
            telemetry.addData("Pinpoint", "ON");
            telemetry.addData("Pinpoint X", String.format("%.1f in", pinpoint.getPosX(DistanceUnit.INCH)));
            telemetry.addData("Pinpoint Y", String.format("%.1f in", pinpoint.getPosY(DistanceUnit.INCH)));
            telemetry.addData("Pinpoint Heading", String.format("%.1f deg", pinpoint.getHeading(AngleUnit.DEGREES)));
        } else
            telemetry.addData("Pinpoint", "OFF");
        telemetry.update();
    }

    private void stopAll() {
        if (leftFront != null)
            leftFront.setPower(0);
        if (rightFront != null)
            rightFront.setPower(0);
        if (leftBack != null)
            leftBack.setPower(0);
        if (rightBack != null)
            rightBack.setPower(0);
        if (intake != null)
            intake.setPower(0);
        if (shooterOne != null)
            shooterOne.setPower(0);
        if (shootertwo != null)
            shootertwo.setPower(0);
        if (servoArm != null)
            servoArm.setPosition(KICKER_DOWN_POS);
    }
}

