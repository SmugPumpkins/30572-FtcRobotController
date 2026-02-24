package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * SPINDEXER ALIGNMENT TEST — Rotation 1 only, CW-only loop
 *
 * goBILDA 5-turn servo: 1800° total, mapped 0.000–1.000
 *   1° = 0.000556 servo units
 *   120° = 0.06667 servo units (one slot gap)
 *
 * ROTATION 1 ONLY — 3 positions (S1, S0, S2).
 * CW-only to eliminate servo backlash from direction reversal.
 *
 * CYCLE PATTERN (RT auto, repeating):
 *   S1 → S0 → S2  (kick at each)
 *   Jump back to S1 (settle, no kick)
 *   S1 → S0 → S2  (kick at each)
 *   ... repeat until RT released
 *
 * CONTROLS:
 *   RT (hold)       Auto-cycle (CW-only loop, repeating)
 *   A               Toggle: MATH positions vs CALIBRATED positions
 *   B               Record current manual position for current slot
 *   Y               Reset to home (S1)
 *   X               Toggle kicker UP/DOWN manually
 *
 *   Left stick Y    Fine manual spindexer adjust (slow)
 *   Right stick Y   Coarse manual spindexer adjust (fast)
 *
 *   RB              Step forward one position (no kick)
 *   LB              Step backward one position (no kick)
 *
 *   D-pad Up/Down   Adjust index settle time (+/- 10ms)
 *   D-pad Rt/Lt     Adjust kick UP (push) time (+/- 10ms)
 */

@TeleOp(name = "Spindexer Align Test", group = "Test")
public class SpindexerAlignTest extends LinearOpMode {

    // ===================== MATH CONSTANTS =====================
    /** S1 home reference — the one known-good calibrated position. */
    private static final double S1_HOME = 0.054;
    /** Servo units per degree (goBILDA 5-turn: 1800° total range). */
    private static final double SERVO_PER_DEG = 1.0 / 1800.0;
    /** Servo units for 120 degrees (one slot gap). */
    private static final double SLOT_GAP = 120.0 * SERVO_PER_DEG;   // 0.06667

    /** Number of positions — Rotation 1 only: S1, S0, S2. */
    private static final int NUM_POSITIONS = 3;

    // ===================== POSITION ARRAYS =====================
    // Rotation 1 only: S1 (offset 0), S0 (offset 1), S2 (offset 2)
    private static final int[] SLOT_OFFSETS = {0, 1, 2};   // offsets × 120° from S1
    private static final int[] SLOT_LABELS  = {1, 0, 2};   // logical slot names

    /** Math-computed positions: S1_HOME + slotOffset × SLOT_GAP */
    private final double[] mathPositions = new double[NUM_POSITIONS];

    /** Calibrated positions from Rev1.java (current stored values, R1 only). */
    private static final double[] calibratedPositions = {
            0.054, 0.119, 0.184   // Rotation 1: S1, S0, S2
    };

    /** Recorded positions from manual adjustment (starts as copy of math). */
    private final double[] recordedPositions = new double[NUM_POSITIONS];
    private final boolean[] hasRecorded = new boolean[NUM_POSITIONS];

    // ===================== KICKER =====================
    private static final double KICKER_UP_POS   = 0.060;
    private static final double KICKER_DOWN_POS  = 0.330;

    // ===================== TIMING =====================
    private int indexMs    = 630;   // spindexer settle time (calibrated)
    private int kickUpMs   = 110;   // kicker push time (calibrated)
    private int kickDownMs = 300;   // kicker retract (safe)

    // ===================== HARDWARE =====================
    private Servo spindexer;
    private Servo servoArm;  // kicker

    // ===================== STATE =====================
    // REWIND = jump back to S1 after finishing S2, settle without kicking
    private enum CycleState { IDLE, INDEXING, KICKER_UP, KICKER_DOWN, REWIND }
    private CycleState state = CycleState.IDLE;
    private ElapsedTime stateTimer = new ElapsedTime();

    private int posIndex = 0;          // current index (0–2)
    private boolean useMathPositions = true;  // true = MATH, false = CALIBRATED
    private boolean autoRunning = false;

    // CW-only loop counter
    private int cwRounds = 0;

    // Manual override
    private boolean manualOverride = false;
    private double manualServoPos = 0;
    private boolean kickerManualUp = false;

    // Stats
    private int totalKicks = 0;
    private ElapsedTime totalTimer = new ElapsedTime();

    // Telemetry
    private double lastCmdPos = 0;

    // Edge detection
    private boolean prevRT, prevA, prevB, prevX, prevY, prevRB, prevLB;
    private boolean prevDU, prevDD, prevDR, prevDL;

    @Override
    public void runOpMode() {
        // ---- Compute math positions (R1 only) ----
        for (int i = 0; i < NUM_POSITIONS; i++) {
            mathPositions[i] = S1_HOME + (SLOT_OFFSETS[i] * SLOT_GAP);
            mathPositions[i] = Math.round(mathPositions[i] * 10000.0) / 10000.0;
            recordedPositions[i] = mathPositions[i];
            hasRecorded[i] = false;
        }

        telemetry.addData("Status", "Initializing...");
        telemetry.addLine("R1 ONLY — 3 positions, CW-only loop");
        telemetry.addLine("Math: S1=" + String.format("%.4f", mathPositions[0])
                + "  S0=" + String.format("%.4f", mathPositions[1])
                + "  S2=" + String.format("%.4f", mathPositions[2]));
        telemetry.addLine("120° gap = " + String.format("%.5f servo", SLOT_GAP));
        telemetry.update();

        // ---- Init hardware ----
        try {
            spindexer = hardwareMap.get(Servo.class, "spindexer");
            if (spindexer instanceof ServoImplEx)
                ((ServoImplEx) spindexer).setPwmRange(new PwmControl.PwmRange(500, 2500));
        } catch (Exception e) { spindexer = null; }

        try {
            servoArm = hardwareMap.get(Servo.class, "servoArm");
            if (servoArm instanceof ServoImplEx)
                ((ServoImplEx) servoArm).setPwmRange(new PwmControl.PwmRange(500, 2500));
        } catch (Exception e) { servoArm = null; }

        // Flush to home (S1)
        posIndex = 0;
        if (spindexer != null) {
            spindexer.setPosition(getActivePosition(0));
            lastCmdPos = getActivePosition(0);
        }
        if (servoArm != null) {
            servoArm.setPosition(KICKER_DOWN_POS);
        }
        sleep(500);

        telemetry.addData("Status", "Ready! RT=Auto cycle | A=Toggle MATH/CAL");
        telemetry.update();
        waitForStart();

        totalTimer.reset();

        while (opModeIsActive()) {
            handleInputs();
            updateCycle();
            displayTelemetry();
        }

        // Safe stop: return home
        if (servoArm != null) servoArm.setPosition(KICKER_DOWN_POS);
        if (spindexer != null) spindexer.setPosition(getActivePosition(0));
    }

    // ===================== INPUT HANDLING =====================

    private void handleInputs() {
        boolean rtNow = gamepad1.right_trigger > 0.5;
        boolean aNow = gamepad1.a;
        boolean bNow = gamepad1.b;
        boolean xNow = gamepad1.x;
        boolean yNow = gamepad1.y;
        boolean rbNow = gamepad1.right_bumper;
        boolean lbNow = gamepad1.left_bumper;
        boolean duNow = gamepad1.dpad_up;
        boolean ddNow = gamepad1.dpad_down;
        boolean drNow = gamepad1.dpad_right;
        boolean dlNow = gamepad1.dpad_left;

        // --- RT: start/stop auto cycle (CW-only loop) ---
        if (rtNow && !prevRT && state == CycleState.IDLE) {
            autoRunning = true;
            posIndex = 0;    // start at S1
            cwRounds = 0;
            manualOverride = false;
            state = CycleState.INDEXING;
            stateTimer.reset();
            totalKicks = 0;
            totalTimer.reset();
        }
        if (!rtNow && autoRunning) {
            autoRunning = false;  // stop after current kick finishes
        }

        // --- A: toggle MATH vs CALIBRATED positions ---
        if (aNow && !prevA && state == CycleState.IDLE) {
            useMathPositions = !useMathPositions;
            manualOverride = false;
            commandSpindexer();
        }

        // --- B: record current manual position for this slot ---
        if (bNow && !prevB && state == CycleState.IDLE && manualOverride) {
            recordedPositions[posIndex] = manualServoPos;
            hasRecorded[posIndex] = true;
        }

        // --- Y: reset to home ---
        if (yNow && !prevY && state == CycleState.IDLE) {
            posIndex = 0;
            manualOverride = false;
            commandSpindexer();
            if (servoArm != null) servoArm.setPosition(KICKER_DOWN_POS);
            kickerManualUp = false;
        }

        // --- X: manual kicker toggle ---
        if (xNow && !prevX && state == CycleState.IDLE) {
            kickerManualUp = !kickerManualUp;
            if (servoArm != null) {
                servoArm.setPosition(kickerManualUp ? KICKER_UP_POS : KICKER_DOWN_POS);
            }
        }

        // --- RB: step forward (wraps 0→1→2→0) ---
        if (rbNow && !prevRB && state == CycleState.IDLE) {
            manualOverride = false;
            posIndex = (posIndex + 1) % NUM_POSITIONS;
            commandSpindexer();
        }

        // --- LB: step backward (wraps 2→1→0→2) ---
        if (lbNow && !prevLB && state == CycleState.IDLE) {
            manualOverride = false;
            posIndex = (posIndex - 1 + NUM_POSITIONS) % NUM_POSITIONS;
            commandSpindexer();
        }

        // --- D-pad Up/Down: index settle time (10ms steps) ---
        if (duNow && !prevDU) indexMs = Math.min(2000, indexMs + 10);
        if (ddNow && !prevDD) indexMs = Math.max(10, indexMs - 10);

        // --- D-pad Right/Left: kick UP time (10ms steps) ---
        if (drNow && !prevDR) kickUpMs = Math.min(1000, kickUpMs + 10);
        if (dlNow && !prevDL) kickUpMs = Math.max(10, kickUpMs - 10);

        // --- Left stick Y: fine manual adjust ---
        if (state == CycleState.IDLE && Math.abs(gamepad1.left_stick_y) > 0.05) {
            if (!manualOverride) {
                manualOverride = true;
                manualServoPos = lastCmdPos;
            }
            manualServoPos += (-gamepad1.left_stick_y * 0.0005);  // very fine
            manualServoPos = Math.max(0.0, Math.min(1.0, manualServoPos));
            if (spindexer != null) {
                spindexer.setPosition(manualServoPos);
                lastCmdPos = manualServoPos;
            }
        }

        // --- Right stick Y: coarse manual adjust ---
        if (state == CycleState.IDLE && Math.abs(gamepad1.right_stick_y) > 0.05) {
            if (!manualOverride) {
                manualOverride = true;
                manualServoPos = lastCmdPos;
            }
            manualServoPos += (-gamepad1.right_stick_y * 0.003);  // faster
            manualServoPos = Math.max(0.0, Math.min(1.0, manualServoPos));
            if (spindexer != null) {
                spindexer.setPosition(manualServoPos);
                lastCmdPos = manualServoPos;
            }
        }

        prevRT = rtNow; prevA = aNow; prevB = bNow; prevX = xNow; prevY = yNow;
        prevRB = rbNow; prevLB = lbNow;
        prevDU = duNow; prevDD = ddNow; prevDR = drNow; prevDL = dlNow;
    }

    // ===================== CYCLE STATE MACHINE =====================

    private void updateCycle() {
        if (spindexer == null || servoArm == null) return;

        switch (state) {
            case INDEXING:
                commandSpindexer();
                servoArm.setPosition(KICKER_DOWN_POS);
                if (stateTimer.milliseconds() >= indexMs) {
                    state = CycleState.KICKER_UP;
                    stateTimer.reset();
                }
                break;

            case KICKER_UP:
                servoArm.setPosition(KICKER_UP_POS);
                commandSpindexer();  // hold position
                if (stateTimer.milliseconds() >= kickUpMs) {
                    state = CycleState.KICKER_DOWN;
                    stateTimer.reset();
                }
                break;

            case KICKER_DOWN:
                servoArm.setPosition(KICKER_DOWN_POS);
                commandSpindexer();  // hold position
                if (stateTimer.milliseconds() >= kickDownMs) {
                    totalKicks++;

                    if (autoRunning) {
                        advanceAutoCycle();
                    } else {
                        state = CycleState.IDLE;
                    }
                }
                break;

            case REWIND:
                // Jump back to S1 — settle without kicking, then start next CW round
                posIndex = 0;
                commandSpindexer();
                servoArm.setPosition(KICKER_DOWN_POS);
                if (stateTimer.milliseconds() >= indexMs) {
                    // Settled at S1 — begin next CW round by kicking here
                    state = CycleState.KICKER_UP;
                    stateTimer.reset();
                }
                break;

            case IDLE:
            default:
                break;
        }
    }

    /**
     * Advance the auto-cycle: CW-only loop within R1 (3 positions).
     * S1 → S0 → S2 (kick at each), then REWIND to S1 and repeat.
     * Always approaches each slot from the same CW direction — no backlash.
     */
    private void advanceAutoCycle() {
        if (posIndex < NUM_POSITIONS - 1) {
            // More slots in this CW round: advance to next
            posIndex++;
            state = CycleState.INDEXING;
            stateTimer.reset();
        } else {
            // Finished CW round (just kicked at S2)
            cwRounds++;
            // Rewind to S1 (settle, then kick)
            state = CycleState.REWIND;
            stateTimer.reset();
        }
    }

    // ===================== HELPERS =====================

    /** Get the position to use based on current mode (MATH or CALIBRATED). */
    private double getActivePosition(int idx) {
        return useMathPositions ? mathPositions[idx] : calibratedPositions[idx];
    }

    private void commandSpindexer() {
        if (spindexer != null && !manualOverride) {
            double pos = getActivePosition(posIndex);
            spindexer.setPosition(pos);
            lastCmdPos = pos;
        }
    }

    /** Convert servo position difference to degrees. */
    private double toDegrees(double servoDiff) {
        return servoDiff / SERVO_PER_DEG;
    }

    // ===================== TELEMETRY =====================

    private void displayTelemetry() {
        telemetry.addLine("===== SPINDEXER ALIGN (R1 ONLY) =====");
        telemetry.addLine("");

        // ---- STATE ----
        String stateStr = state.toString();
        if (autoRunning && state == CycleState.REWIND) {
            stateStr = "REWIND to S1...";
        } else if (autoRunning) {
            stateStr += " [CW >>]";
        }
        telemetry.addData("STATE", stateStr);
        telemetry.addData("MODE", useMathPositions ? ">>> MATH (120 deg exact) <<<" : ">>> CALIBRATED (stored) <<<");
        telemetry.addData("CW Rounds", cwRounds);
        telemetry.addLine("");

        // ---- CURRENT POSITION ----
        telemetry.addLine("--- CURRENT POSITION ---");
        telemetry.addData("Index", posIndex + "/" + (NUM_POSITIONS - 1));
        telemetry.addData("Slot", "S" + SLOT_LABELS[posIndex]);
        telemetry.addData("Direction", "CW only (S1>S0>S2, rewind)");
        telemetry.addLine("");

        double mathPos = mathPositions[posIndex];
        double calPos  = calibratedPositions[posIndex];
        double errDeg  = toDegrees(calPos - mathPos);

        telemetry.addData("MATH position", String.format("%.4f  (%.1f deg)", mathPos, toDegrees(mathPos)));
        telemetry.addData("CALIBRATED pos", String.format("%.4f  (%.1f deg)", calPos, toDegrees(calPos)));
        telemetry.addData("Cal vs Math err", String.format("%+.4f  (%+.1f deg)", calPos - mathPos, errDeg));
        telemetry.addData("COMMANDED", String.format("%.4f  (%.1f deg)", lastCmdPos, toDegrees(lastCmdPos)));

        if (manualOverride) {
            double manualVsMath = toDegrees(manualServoPos - mathPos);
            double manualVsCal = toDegrees(manualServoPos - calPos);
            telemetry.addLine("** MANUAL OVERRIDE **");
            telemetry.addData("Manual vs Math", String.format("%+.4f  (%+.1f deg)", manualServoPos - mathPos, manualVsMath));
            telemetry.addData("Manual vs Cal", String.format("%+.4f  (%+.1f deg)", manualServoPos - calPos, manualVsCal));
            telemetry.addLine("  Press B to record this position");
        }

        if (hasRecorded[posIndex]) {
            telemetry.addData("RECORDED", String.format("%.4f", recordedPositions[posIndex]));
        }
        telemetry.addLine("");

        // ---- SLOT SPACING CHECK ----
        telemetry.addLine("--- SLOT SPACING (should be 120 deg = " + String.format("%.5f", SLOT_GAP) + ") ---");
        double gap01 = getActivePosition(1) - getActivePosition(0);
        double gap12 = getActivePosition(2) - getActivePosition(1);
        double gap02 = getActivePosition(2) - getActivePosition(0);
        telemetry.addData("S1 to S0",
                String.format("%.5f = %.1f deg %s", gap01, toDegrees(gap01),
                        Math.abs(toDegrees(gap01) - 120.0) < 1.0 ? "OK" : "OFF BY " + String.format("%+.1f deg", toDegrees(gap01) - 120.0)));
        telemetry.addData("S0 to S2",
                String.format("%.5f = %.1f deg %s", gap12, toDegrees(gap12),
                        Math.abs(toDegrees(gap12) - 120.0) < 1.0 ? "OK" : "OFF BY " + String.format("%+.1f deg", toDegrees(gap12) - 120.0)));
        telemetry.addData("S1 to S2",
                String.format("%.5f = %.1f deg %s", gap02, toDegrees(gap02),
                        Math.abs(toDegrees(gap02) - 240.0) < 1.0 ? "OK" : "OFF BY " + String.format("%+.1f deg", toDegrees(gap02) - 240.0)));
        telemetry.addLine("");

        // ---- FULL POSITION TABLE ----
        telemetry.addLine("--- POSITION TABLE ---");
        StringBuilder table = new StringBuilder();
        for (int i = 0; i < NUM_POSITIONS; i++) {
            String marker = (i == posIndex) ? " <<" : "";
            String recStr = hasRecorded[i] ? String.format("%.4f", recordedPositions[i]) : " ----";
            table.append(String.format("  [%d] S%d  Math:%.4f  Cal:%.4f  Rec:%s  err:%+.1f deg%s\n",
                    i, SLOT_LABELS[i],
                    mathPositions[i], calibratedPositions[i], recStr,
                    toDegrees(calibratedPositions[i] - mathPositions[i]),
                    marker));
        }
        telemetry.addLine(table.toString());

        // ---- TIMING ----
        telemetry.addLine("--- TIMING (D-pad, 10ms steps) ---");
        telemetry.addData("Index settle (D-Up/Dn)", indexMs + " ms  << spindexer wait before kick");
        telemetry.addData("Kick UP      (D-Rt/Lt)", kickUpMs + " ms  << push duration");
        telemetry.addData("Kick DOWN    (fixed)", kickDownMs + " ms  << retract before next move");
        int total = indexMs + kickUpMs + kickDownMs;
        telemetry.addData("Total per kick", total + " ms  (" + String.format("%.1f", 1000.0 / total) + " balls/sec)");
        telemetry.addLine("");

        // ---- STATS ----
        telemetry.addData("Total kicks", totalKicks);
        if (totalKicks > 0) {
            telemetry.addData("Avg ms/kick", String.format("%.0f", totalTimer.milliseconds() / totalKicks));
        }
        telemetry.addLine("");

        // ---- RECORDED ARRAY ----
        if (anyRecorded()) {
            telemetry.addLine("--- RECORDED (copy into Rev1.java) ---");
            StringBuilder arr = new StringBuilder("{ ");
            for (int i = 0; i < NUM_POSITIONS; i++) {
                double val = hasRecorded[i] ? recordedPositions[i] : mathPositions[i];
                arr.append(String.format("%.4f", val));
                if (i < NUM_POSITIONS - 1) arr.append(", ");
            }
            arr.append(" }  // R1: S1, S0, S2");
            telemetry.addLine(arr.toString());
        }
        telemetry.addLine("");

        // ---- CONTROLS ----
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("RT=Auto CW loop | A=Math/Cal toggle");
        telemetry.addLine("RB/LB=Step | Y=Home | X=Kicker toggle");
        telemetry.addLine("LStick=Fine adj | RStick=Coarse adj | B=Record");
        telemetry.addLine("D-Up/Dn=Index time | D-Rt/Lt=Kick UP time");

        telemetry.update();
    }

    private boolean anyRecorded() {
        for (boolean r : hasRecorded) if (r) return true;
        return false;
    }
}

