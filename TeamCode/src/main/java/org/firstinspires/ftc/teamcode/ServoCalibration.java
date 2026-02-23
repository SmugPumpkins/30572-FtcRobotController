package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * SERVO CALIBRATION - Find hood, kicker, and spindexer positions
 *
 * Use this OpMode to discover correct values for CompetitionTeleOp constants.
 * Record the values that work and update HOOD_POS_NEAR, HOOD_POS_FAR,
 * KICKER_UP_POS, KICKER_DOWN_POS, SPINDEXER_POS_0/1/2 in CompetitionTeleOp.java
 *
 * CONTROLS:
 *   A + D-pad Up/Down:   Spindexer adjust (step 0.01)
 *   B + D-pad Up/Down:   Hood adjust (step 0.02)
 *   X + D-pad Up/Down:   Kicker adjust (step 0.01)
 *   D-pad Up (no mod):   Spindexer preset 0
 *   D-pad Right (no mod): Spindexer preset 0.333
 *   D-pad Down (no mod): Spindexer preset 0.666
 */
@TeleOp(name = "Servo Calibration", group = "Utility")
public class ServoCalibration extends LinearOpMode {

    private Servo hoodOne, hood, servoArm, spindexer;

    private double hoodPosition = 0.5;
    private double kickerPosition = 0.2;
    private double spindexerPosition = 0.0;

    private static final double SPINDEXER_STEP = 0.009;
    private static final double HOOD_STEP = 0.02;
    private static final double KICKER_STEP = 0.01;

    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        hoodOne = null;
        hood = null;
        servoArm = null;
        spindexer = null;
        try { hoodOne = hardwareMap.get(Servo.class, "hoodOne"); } catch (Exception e) { telemetry.addData("ERROR", "hoodOne not found"); }
        try { hood = hardwareMap.get(Servo.class, "hood"); } catch (Exception e) { telemetry.addData("ERROR", "hood not found"); }
        try { servoArm = hardwareMap.get(Servo.class, "servoArm"); } catch (Exception e) { telemetry.addData("ERROR", "servoArm not found"); }
        try { spindexer = hardwareMap.get(Servo.class, "spindexer"); } catch (Exception e) { telemetry.addData("ERROR", "spindexer not found"); }

        // GoBILDA 300 deg servos: full range 500-2500 usec
        if (spindexer instanceof ServoImplEx) {
            ((ServoImplEx) spindexer).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
        if (hoodOne instanceof ServoImplEx) {
            ((ServoImplEx) hoodOne).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
        if (hood != null && hood instanceof ServoImplEx) {
            ((ServoImplEx) hood).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }
        if (servoArm instanceof ServoImplEx) {
            ((ServoImplEx) servoArm).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }

        telemetry.addData("Status", "Ready. A=Spindexer B=Hood X=Kicker + D-pad Up/Down");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            boolean dpadRight = gamepad1.dpad_right;
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;

            // Edge detection: only act on press, not hold
            boolean dpadUpPressed = dpadUp && !prevDpadUp;
            boolean dpadDownPressed = dpadDown && !prevDpadDown;
            prevDpadUp = dpadUp;
            prevDpadDown = dpadDown;

            // No modifier: D-pad = spindexer presets
            if (!a && !b && !x) {
                if (dpadUp) spindexerPosition = 0.0;
                if (dpadRight) spindexerPosition = 0.333;
                if (dpadDown) spindexerPosition = 0.666;
            } else {
                // A = spindexer, B = hood, X = kicker (priority: A > B > X)
                if (a && spindexer != null) {
                    if (dpadUpPressed) { spindexerPosition += SPINDEXER_STEP; if (spindexerPosition > 1) spindexerPosition = 1; }
                    if (dpadDownPressed) { spindexerPosition -= SPINDEXER_STEP; if (spindexerPosition < 0) spindexerPosition = 0; }
                } else if (b) {
                    if (dpadUpPressed) { hoodPosition += HOOD_STEP; if (hoodPosition > 1) hoodPosition = 1; }
                    if (dpadDownPressed) { hoodPosition -= HOOD_STEP; if (hoodPosition < 0) hoodPosition = 0; }
                } else if (x) {
                    if (dpadUpPressed) { kickerPosition += KICKER_STEP; if (kickerPosition > 1) kickerPosition = 1; }
                    if (dpadDownPressed) { kickerPosition -= KICKER_STEP; if (kickerPosition < 0) kickerPosition = 0; }
                }
            }

            // Apply servo positions
            if (spindexer != null) spindexer.setPosition(spindexerPosition);
            if (hoodOne != null && hood != null) {
                hoodOne.setPosition(hoodPosition);
                hood.setPosition(1 - hoodPosition);
            }
            if (servoArm != null) servoArm.setPosition(kickerPosition);

            // Telemetry - always show current values to record
            telemetry.addData("--- RECORD THESE VALUES ---", "");
            telemetry.addData("Spindexer", String.format("%.3f", spindexerPosition));
            telemetry.addData("  -> SPINDEXER_POS_0/1/2", "align 3 slots with kick plate");
            telemetry.addData("Hood", String.format("%.3f", hoodPosition));
            telemetry.addData("  -> HOOD_POS_NEAR/FAR", "use for near/far shots");
            telemetry.addData("Kicker", String.format("%.3f", kickerPosition));
            telemetry.addData("  -> KICKER_UP_POS, KICKER_DOWN_POS", "0.0 = touching ball");
            telemetry.addData("Controls", "A+Up/Down=Spindexer B=Hood X=Kicker");
            telemetry.update();
        }
    }
}


