package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * SHOOTER CALIBRATION TEST
 *
 * Stand at known distances, adjust hood and velocity until shots land consistently,
 * then press A to record the (ty, velocity, hood) data point. Uses full spindexer
 * cycling (CW-only) so you can shoot all 3 balls without reloading.
 *
 * CONTROLS:
 *   D-pad Up/Down:     Hood coarse +/-0.05
 *   D-pad Right/Left:  Velocity coarse +/-100 tks/s
 *   Right stick Y:     Hood fine adjust (continuous)
 *   Left stick Y:      Velocity fine adjust (continuous)
 *   LT (hold):         Spin flywheel only (no shoot) - for RPM tuning
 *   RT (hold):         Shoot cycle (flywheel + kick + advance)
 *   A:                 Record data point (ty, velocity, hood)
 *   B:                 Clear all recorded data points
 *   RB / LB:           Spindexer step forward / backward
 *   Y:                 Reset spindexer to home (S1)
 *   X:                 Toggle Limelight pipeline (0-9)
 *
 * DISPLAY:
 *   - Target vs Actual RPM, tks/s, and % of motor max
 *   - Hood position (internal 0-1 and servo value)
 *   - Limelight tx, ty, ta; computed distance (LL and real)
 *   - What Rev1's current anchor table would compute for this ty
 *   - Recorded data points table (auto-sorted by ty)
 *   - Ready-to-paste arrays for Rev1.java
 */

@TeleOp(name = "Shooter Calibration Test", group = "Test")
public class ShooterCalibrationTest extends LinearOpMode {

    // ===================== CONSTANTS (match Rev1) =====================

    // Hood limits (5-turn torque servos, inverted: lower value = higher position)
    private static final double HOOD_POS_TOP = 0.180;
    private static final double HOOD_POS_BOTTOM = 0.500;

    // Motor limits (goBILDA 5202 bare 6000RPM, 28 CPR, 1:1)
    private static final double MOTOR_CPR = 28.0;
    private static final double MOTOR_MAX_TKS = 2800.0;  // ~6000RPM × 28/60 @ 12V
    private static final double VELOCITY_MIN = 0;
    private static final double VELOCITY_MAX = 3500;      // allow over-max for fresh battery

    // Kicker (calibrated — same as Rev1)
    private static final double KICKER_UP_POS = 0.060;
    private static final double KICKER_DOWN_POS = 0.330;
    private static final int KICKER_UP_MS = 110;
    private static final int KICKER_DOWN_MS = 300;

    // Spindexer (CW-only, R1 only — same as Rev1)
    // !! WARNING: CW-ONLY !! Never go CCW — servo backlash causes misalignment.
    private static final int SPINDEXER_INDEX_MS = 630;
    private static final double[] SPINDEXER_SHOOT_ALL = {
            0.054, 0.119, 0.184    // R1: S1, S0, S2
    };
    private static final int[] SPINDEXER_SLOT_MAP = {1, 0, 2};

    // Camera calibration (measured — same as Rev1)
    private static final double VISION_TAG_HEIGHT_IN = 38.7;
    private static final double VISION_CAMERA_HEIGHT_IN = 14.875;
    private static final double VISION_CAMERA_TILT_DEG = 24.0;
    private static final double DISTANCE_CORRECTION = 0.95;

    // Rev1's current anchor table (for comparison display)
    private static final double[] REV1_ANCHOR_TY  = { -14,   -10,   -7,    -2,    3,     10   };
    private static final double[] REV1_TY_VEL     = { 2800,  2750,  2693,  2350,  1950,  1600 };
    private static final double[] REV1_TY_HOOD    = { 0.20,  0.10,  0.05,  0.08,  0.25,  0.55 };

    // ===================== HARDWARE =====================
    private DcMotorEx shooterOne, shootertwo;
    private Servo hoodOne, hood;
    private Servo spindexer, servoArm;
    private Limelight3A limelight;
    private boolean limelightEnabled = false;

    // ===================== STATE =====================
    private double hoodPosition = 0.5;       // 0=flat, 1=steep
    private double targetVelocity = 2693;    // start at calibrated mid-range
    private int llPipeline = 0;
    private int shootPosIndex = 0;

    // Shoot state machine
    private enum ShootState { IDLE, INDEXING, KICKER_UP, KICKER_DOWN }
    private ShootState shootState = ShootState.IDLE;
    private ElapsedTime stateTimer = new ElapsedTime();
    private boolean flywheelOn = false;      // true when LT or RT held

    // Recorded calibration data: each entry = { ty, velocity, hood }
    private ArrayList<double[]> recordedPoints = new ArrayList<>();

    // Edge detection
    private boolean prevDU, prevDD, prevDL, prevDR;
    private boolean prevA, prevB, prevX, prevY;
    private boolean prevRT, prevRB, prevLB;

    // ===================== LIFECYCLE =====================

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing Shooter Calibration...");
        telemetry.update();

        initHardware();

        // Flush spindexer to home
        if (spindexer != null) {
            shootPosIndex = 0;
            spindexer.setPosition(SPINDEXER_SHOOT_ALL[0]);
            sleep(500);
        }

        telemetry.addData("Status", "Ready!");
        telemetry.addLine("LT=Spin flywheel | RT=Shoot | D-pad=Adjust | A=Record");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            handleInputs();
            updateShootCycle();
            updateHood();
            displayTelemetry();
        }

        // Safe stop
        if (shooterOne != null) shooterOne.setPower(0);
        if (shootertwo != null) shootertwo.setPower(0);
        if (servoArm != null) servoArm.setPosition(KICKER_DOWN_POS);
    }

    // ===================== HARDWARE INIT =====================

    private void initHardware() {
        try {
            shooterOne = hardwareMap.get(DcMotorEx.class, "shooterOne");
            shootertwo = hardwareMap.get(DcMotorEx.class, "shooterTwo");
            shooterOne.setDirection(DcMotor.Direction.REVERSE);
            shootertwo.setDirection(DcMotor.Direction.FORWARD);
            shooterOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shootertwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            PIDFCoefficients pidf = new PIDFCoefficients(300, 0, 0, 10);
            shooterOne.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            shootertwo.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        } catch (Exception e) {
            shooterOne = null;
            shootertwo = null;
        }

        try {
            hoodOne = hardwareMap.get(Servo.class, "hoodOne");
            hood = hardwareMap.get(Servo.class, "hood");
            if (hoodOne instanceof ServoImplEx)
                ((ServoImplEx) hoodOne).setPwmRange(new PwmControl.PwmRange(500, 2500));
            if (hood instanceof ServoImplEx)
                ((ServoImplEx) hood).setPwmRange(new PwmControl.PwmRange(500, 2500));
        } catch (Exception e) {
            hoodOne = null;
            hood = null;
        }

        try {
            spindexer = hardwareMap.get(Servo.class, "spindexer");
            servoArm = hardwareMap.get(Servo.class, "servoArm");
            if (spindexer instanceof ServoImplEx)
                ((ServoImplEx) spindexer).setPwmRange(new PwmControl.PwmRange(500, 2500));
            if (servoArm instanceof ServoImplEx)
                ((ServoImplEx) servoArm).setPwmRange(new PwmControl.PwmRange(500, 2500));
            servoArm.setPosition(KICKER_DOWN_POS);
        } catch (Exception e) {
            spindexer = null;
            servoArm = null;
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
    }

    // ===================== INPUT HANDLING =====================

    private void handleInputs() {
        boolean duNow = gamepad1.dpad_up;
        boolean ddNow = gamepad1.dpad_down;
        boolean dlNow = gamepad1.dpad_left;
        boolean drNow = gamepad1.dpad_right;
        boolean aNow = gamepad1.a;
        boolean bNow = gamepad1.b;
        boolean xNow = gamepad1.x;
        boolean yNow = gamepad1.y;
        boolean rtNow = gamepad1.right_trigger > 0.5;
        boolean ltNow = gamepad1.left_trigger > 0.5;
        boolean rbNow = gamepad1.right_bumper;
        boolean lbNow = gamepad1.left_bumper;

        // --- D-pad: coarse adjustment ---
        if (duNow && !prevDU) hoodPosition = Math.min(1.0, hoodPosition + 0.05);
        if (ddNow && !prevDD) hoodPosition = Math.max(0.0, hoodPosition - 0.05);
        if (drNow && !prevDR) targetVelocity = Math.min(VELOCITY_MAX, targetVelocity + 100);
        if (dlNow && !prevDL) targetVelocity = Math.max(VELOCITY_MIN, targetVelocity - 100);

        // --- Left stick Y: fine velocity adjust ---
        if (Math.abs(gamepad1.left_stick_y) > 0.05) {
            targetVelocity += (-gamepad1.left_stick_y * 3.0);  // ~3 tks/s per frame
            targetVelocity = Math.max(VELOCITY_MIN, Math.min(VELOCITY_MAX, targetVelocity));
        }

        // --- Right stick Y: fine hood adjust ---
        if (Math.abs(gamepad1.right_stick_y) > 0.05) {
            hoodPosition += (-gamepad1.right_stick_y * 0.003);  // fine
            hoodPosition = Math.max(0.0, Math.min(1.0, hoodPosition));
        }

        // --- LT: continuous flywheel (no shoot) ---
        if (ltNow && shootState == ShootState.IDLE) {
            flywheelOn = true;
        }
        if (!ltNow && !rtNow && shootState == ShootState.IDLE) {
            flywheelOn = false;
        }

        // --- RT: start shoot cycle ---
        if (rtNow && !prevRT && shootState == ShootState.IDLE) {
            flywheelOn = true;
            shootState = ShootState.INDEXING;
            stateTimer.reset();
        }
        // Release RT: stop shooting (flywheel stays on if LT held)
        if (!rtNow && shootState != ShootState.IDLE) {
            shootState = ShootState.IDLE;
            if (servoArm != null) servoArm.setPosition(KICKER_DOWN_POS);
            if (!ltNow) flywheelOn = false;
        }

        // --- RB: spindexer step forward (CW-only) ---
        if (rbNow && !prevRB && shootState == ShootState.IDLE) {
            shootPosIndex = (shootPosIndex + 1) % SPINDEXER_SHOOT_ALL.length;
            if (spindexer != null) spindexer.setPosition(SPINDEXER_SHOOT_ALL[shootPosIndex]);
        }
        // --- LB: spindexer step backward ---
        if (lbNow && !prevLB && shootState == ShootState.IDLE) {
            shootPosIndex = (shootPosIndex - 1 + SPINDEXER_SHOOT_ALL.length) % SPINDEXER_SHOOT_ALL.length;
            if (spindexer != null) spindexer.setPosition(SPINDEXER_SHOOT_ALL[shootPosIndex]);
        }

        // --- A: record data point ---
        if (aNow && !prevA) {
            double ty = getLimelightTy();
            recordedPoints.add(new double[]{ ty, targetVelocity, hoodPosition });
            // Auto-sort by ty ascending
            Collections.sort(recordedPoints, new Comparator<double[]>() {
                @Override
                public int compare(double[] a, double[] b) {
                    return Double.compare(a[0], b[0]);
                }
            });
        }

        // --- B: clear all recorded points ---
        if (bNow && !prevB) {
            recordedPoints.clear();
        }

        // --- X: cycle Limelight pipeline ---
        if (xNow && !prevX) {
            llPipeline = (llPipeline + 1) % 10;
            if (limelight != null) limelight.pipelineSwitch(llPipeline);
        }

        // --- Y: reset spindexer to home ---
        if (yNow && !prevY && shootState == ShootState.IDLE) {
            shootPosIndex = 0;
            if (spindexer != null) spindexer.setPosition(SPINDEXER_SHOOT_ALL[0]);
            if (servoArm != null) servoArm.setPosition(KICKER_DOWN_POS);
        }

        prevDU = duNow; prevDD = ddNow; prevDL = dlNow; prevDR = drNow;
        prevA = aNow; prevB = bNow; prevX = xNow; prevY = yNow;
        prevRT = rtNow; prevRB = rbNow; prevLB = lbNow;
    }

    // ===================== SHOOT STATE MACHINE =====================

    private void updateShootCycle() {
        // Flywheel control
        if (shooterOne != null && shootertwo != null) {
            if (flywheelOn) {
                shooterOne.setVelocity(targetVelocity);
                shootertwo.setVelocity(targetVelocity);
            } else {
                shooterOne.setPower(0);
                shootertwo.setPower(0);
            }
        }

        // Kicker safety
        if (servoArm != null && shootState != ShootState.KICKER_UP) {
            servoArm.setPosition(KICKER_DOWN_POS);
        }

        if (spindexer == null || servoArm == null) return;

        switch (shootState) {
            case INDEXING:
                spindexer.setPosition(SPINDEXER_SHOOT_ALL[shootPosIndex]);
                if (stateTimer.milliseconds() >= SPINDEXER_INDEX_MS) {
                    shootState = ShootState.KICKER_UP;
                    stateTimer.reset();
                }
                break;

            case KICKER_UP:
                spindexer.setPosition(SPINDEXER_SHOOT_ALL[shootPosIndex]);
                servoArm.setPosition(KICKER_UP_POS);
                if (stateTimer.milliseconds() >= KICKER_UP_MS) {
                    shootState = ShootState.KICKER_DOWN;
                    stateTimer.reset();
                }
                break;

            case KICKER_DOWN:
                servoArm.setPosition(KICKER_DOWN_POS);
                spindexer.setPosition(SPINDEXER_SHOOT_ALL[shootPosIndex]);
                if (stateTimer.milliseconds() >= KICKER_DOWN_MS) {
                    // Advance CW-only, rewind after S2
                    shootPosIndex = (shootPosIndex + 1) % SPINDEXER_SHOOT_ALL.length;
                    shootState = ShootState.INDEXING;
                    stateTimer.reset();
                }
                break;

            case IDLE:
            default:
                spindexer.setPosition(SPINDEXER_SHOOT_ALL[shootPosIndex]);
                break;
        }
    }

    // ===================== HOOD =====================

    private void updateHood() {
        if (hoodOne == null || hood == null) return;
        double servoPos = mapHoodToServo(hoodPosition);
        hoodOne.setPosition(servoPos);
        hood.setPosition(HOOD_POS_TOP + HOOD_POS_BOTTOM - servoPos);  // inverted
    }

    private double mapHoodToServo(double hoodPos) {
        hoodPos = Math.max(0.0, Math.min(1.0, hoodPos));
        return HOOD_POS_BOTTOM - (hoodPos * (HOOD_POS_BOTTOM - HOOD_POS_TOP));
    }

    // ===================== VISION HELPERS =====================

    private double getLimelightTy() {
        if (!limelightEnabled || limelight == null) return 0;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) return result.getTy();
        return 0;
    }

    private double getLimelightTx() {
        if (!limelightEnabled || limelight == null) return 0;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) return result.getTx();
        return 0;
    }

    private double distanceFromTy(double tyDeg) {
        double heightDiff = VISION_TAG_HEIGHT_IN - VISION_CAMERA_HEIGHT_IN;
        double totalAngleDeg = VISION_CAMERA_TILT_DEG + tyDeg;
        if (totalAngleDeg < 0.5 || totalAngleDeg > 60) return -1;
        double dist = heightDiff / Math.tan(Math.toRadians(totalAngleDeg));
        return Math.max(12, Math.min(200, dist));
    }

    /** Interpolate from Rev1's anchor table (for comparison). */
    private double interpolateAnchors(double xVal, double[] xAnchors, double[] yAnchors) {
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

    private double tksToRPM(double tks) {
        return (tks / MOTOR_CPR) * 60.0;
    }

    private double tksPct(double tks) {
        return (tks / MOTOR_MAX_TKS) * 100.0;
    }

    // ===================== TELEMETRY =====================

    private void displayTelemetry() {
        telemetry.addLine("===== SHOOTER CALIBRATION TEST =====");
        telemetry.addLine("");

        // ---- SHOOT STATE ----
        telemetry.addData("State", shootState.toString());
        telemetry.addData("Spindexer", shootPosIndex + "/" + (SPINDEXER_SHOOT_ALL.length - 1)
                + " (S" + SPINDEXER_SLOT_MAP[shootPosIndex] + ")  "
                + String.format("%.3f", SPINDEXER_SHOOT_ALL[shootPosIndex]));
        telemetry.addLine("");

        // ---- FLYWHEEL ----
        telemetry.addLine("--- FLYWHEEL ---");
        telemetry.addData("Flywheel", flywheelOn ? "ON (LT=spin, RT=shoot)" : "OFF");
        telemetry.addData("TARGET", String.format("%.0f tks/s  |  %.0f RPM  |  %.0f%% max",
                targetVelocity, tksToRPM(targetVelocity), tksPct(targetVelocity)));
        if (shooterOne != null) {
            double actual = shooterOne.getVelocity();
            double diff = actual - targetVelocity;
            telemetry.addData("ACTUAL", String.format("%.0f tks/s  |  %.0f RPM  |  %.0f%% max",
                    actual, tksToRPM(actual), tksPct(actual)));
            telemetry.addData("ERROR", String.format("%+.0f tks/s  (%+.0f RPM)", diff, tksToRPM(diff)));
        }
        telemetry.addLine("  LStick=Fine vel  |  D-Rt/Lt=+/-100");
        telemetry.addLine("");

        // ---- HOOD ----
        telemetry.addLine("--- HOOD ---");
        double servoVal = mapHoodToServo(hoodPosition);
        telemetry.addData("HOOD", String.format("%.3f  (0=flat, 1=steep)", hoodPosition));
        telemetry.addData("Servo", String.format("%.3f  (top=%.3f, bot=%.3f)", servoVal, HOOD_POS_TOP, HOOD_POS_BOTTOM));
        telemetry.addLine("  RStick=Fine hood  |  D-Up/Dn=+/-0.05");
        telemetry.addLine("");

        // ---- LIMELIGHT ----
        telemetry.addLine("--- LIMELIGHT ---");
        if (limelightEnabled && limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double ty = result.getTy();
                double tx = result.getTx();
                double ta = result.getTa();
                double distLL = distanceFromTy(ty);
                double distReal = distLL > 0 ? distLL * DISTANCE_CORRECTION : -1;

                telemetry.addData("Pipeline", llPipeline);
                telemetry.addData("ty", String.format("%.2f deg", ty));
                telemetry.addData("tx", String.format("%.2f deg", tx));
                telemetry.addData("ta", String.format("%.1f%%", ta));
                if (distLL > 0) {
                    telemetry.addData("Distance (LL)", String.format("%.1f in", distLL));
                    telemetry.addData("Distance (real)", String.format("%.1f in  (x%.2f)", distReal, DISTANCE_CORRECTION));
                } else {
                    telemetry.addData("Distance", "N/A (angle out of range)");
                }

                // Tags detected
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    StringBuilder ids = new StringBuilder();
                    for (LLResultTypes.FiducialResult f : fiducials) {
                        if (ids.length() > 0) ids.append(", ");
                        ids.append("Tag ").append(f.getFiducialId());
                    }
                    telemetry.addData("Tags", ids.toString());
                }

                // ---- REV1 COMPARISON ----
                telemetry.addLine("");
                telemetry.addLine("--- REV1 WOULD COMPUTE (for ty=" + String.format("%.1f", ty) + ") ---");
                double rev1Vel = interpolateAnchors(ty, REV1_ANCHOR_TY, REV1_TY_VEL);
                double rev1Hood = interpolateAnchors(ty, REV1_ANCHOR_TY, REV1_TY_HOOD);
                telemetry.addData("Rev1 vel", String.format("%.0f tks/s  (%.0f RPM)", rev1Vel, tksToRPM(rev1Vel)));
                telemetry.addData("Rev1 hood", String.format("%.3f", rev1Hood));
                telemetry.addData("YOUR vel", String.format("%.0f tks/s  (diff: %+.0f)", targetVelocity, targetVelocity - rev1Vel));
                telemetry.addData("YOUR hood", String.format("%.3f  (diff: %+.3f)", hoodPosition, hoodPosition - rev1Hood));
            } else {
                telemetry.addData("LimeLight", "NO VALID RESULT");
            }
        } else {
            telemetry.addData("LimeLight", "NOT CONNECTED");
        }
        telemetry.addLine("");

        // ---- TIMING ----
        int totalMs = SPINDEXER_INDEX_MS + KICKER_UP_MS + KICKER_DOWN_MS;
        telemetry.addData("Per-ball",
                "Idx:" + SPINDEXER_INDEX_MS + " Up:" + KICKER_UP_MS + " Dn:" + KICKER_DOWN_MS
                        + " = " + totalMs + "ms (" + String.format("%.1f", 1000.0 / totalMs) + " balls/sec)");
        telemetry.addLine("");

        // ---- RECORDED POINTS ----
        telemetry.addData("Recorded", recordedPoints.size() + " points  (A=record, B=clear)");
        if (!recordedPoints.isEmpty()) {
            telemetry.addLine("--- DATA TABLE (sorted by ty) ---");
            for (int i = 0; i < recordedPoints.size(); i++) {
                double[] pt = recordedPoints.get(i);
                double dist = distanceFromTy(pt[0]);
                telemetry.addLine(String.format("  #%d: ty=%+.1f  vel=%.0f  hood=%.2f  (~%.0fin)",
                        i + 1, pt[0], pt[1], pt[2], dist > 0 ? dist : 0));
            }

            // ---- READY-TO-PASTE ARRAYS ----
            telemetry.addLine("");
            telemetry.addLine("--- PASTE INTO Rev1.java ---");
            StringBuilder tyArr = new StringBuilder("SHOOTER_ANCHOR_TY  = { ");
            StringBuilder velArr = new StringBuilder("SHOOTER_TY_VELOCITY = { ");
            StringBuilder hoodArr = new StringBuilder("SHOOTER_TY_HOOD     = { ");
            for (int i = 0; i < recordedPoints.size(); i++) {
                double[] pt = recordedPoints.get(i);
                if (i > 0) { tyArr.append(", "); velArr.append(", "); hoodArr.append(", "); }
                tyArr.append(String.format("%.1f", pt[0]));
                velArr.append(String.format("%.0f", pt[1]));
                hoodArr.append(String.format("%.2f", pt[2]));
            }
            tyArr.append(" };");
            velArr.append(" };");
            hoodArr.append(" };");
            telemetry.addLine(tyArr.toString());
            telemetry.addLine(velArr.toString());
            telemetry.addLine(hoodArr.toString());
        }
        telemetry.addLine("");

        // ---- CONTROLS ----
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("LT=Spin only | RT=Shoot cycle");
        telemetry.addLine("LStick=Fine vel | RStick=Fine hood");
        telemetry.addLine("D-pad=Coarse adj | A=Record | B=Clear");
        telemetry.addLine("RB/LB=Spindexer | Y=Home | X=Pipeline");

        telemetry.update();
    }
}

