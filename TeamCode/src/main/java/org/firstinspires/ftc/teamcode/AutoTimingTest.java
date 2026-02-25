package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Constants.FORWARD;
import static org.firstinspires.ftc.teamcode.utils.Constants.REVERSE;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDriveTrain;

@TeleOp
public class AutoTimingTest extends OpMode {
    private MecanumDriveTrain drive;
    private int time = 1000;
    private int action = 1;
    private int direction = 1;
    public void forward(long time){
        drive.drive(1, 0, 0);
        try {
            sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        drive.drive(0, 0, 0);
    }
    public void reverse(long time){
        drive.drive(-1, 0, 0);
        try {
            sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        drive.drive(0, 0, 0);
    }
    public void rotate(long time, int direction) {
        drive.drive(0, 0, direction);
        try {
            sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        drive.drive(0, 0, 0);
    }
    public void strafe(long time, int direction){
        drive.drive(0, direction, 0);
        try {
            sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        drive.drive(0, 0, 0);
    }
    public void init() {
        drive = new MecanumDriveTrain(hardwareMap, telemetry);
        drive.init(REVERSE, FORWARD, REVERSE, FORWARD);
    }
    public void loop() {
        telemetry.addLine("Use dpad to select movement and time value");
        telemetry.addLine("Press X to set action as forward");
        telemetry.addLine("Press B to set action as reversed");
        telemetry.addLine("Press A to run action");
            telemetry.addLine("           VVVVVV");
        if (action == 1) {
            telemetry.addLine("Strafe    Forward    Reverse");
        } else if (action == 2) {
            telemetry.addLine("Forward   Reverse    Rotate");
        } else if (action == 3) {
            telemetry.addLine("Reverse   Rotate     Strafe");
        } else if (action == 4) {
            telemetry.addLine("Rotate    Strafe     Forward");
        }
        telemetry.addData("Time value", time);
        if (direction == 1) {
            telemetry.addLine("Direction : Forward (1)");
        } else {
            telemetry.addLine("Direction : Reverse (-1)");
        }
        telemetry.update();
        if (gamepad1.dpad_left) {
            action = action - 1;
        } else if (gamepad1.dpad_right) {
            action = action + 1;
        }
        if (action < 1) {
            action = 4;
        } else if (action > 4) {
            action = 1;
        }
        if (gamepad1.dpad_up) {
            time = time + 100;
        } else if (gamepad1.dpad_down) {
            time = time - 100;
        }
        if (time < 0) {
            time = 100;
        }
        if (gamepad1.xWasPressed()) {
            direction = 1;
        } else if (gamepad1.bWasPressed()) {
            direction = -1;
        }
        if (gamepad1.aWasPressed()) {
            if (action == 1) {
                telemetry.addLine("Running action 'Forward' for " + time + " milliseconds");
                telemetry.update();
                forward(time);
            } else if (action == 2) {
                telemetry.addLine("Running action 'Reverse' for " + time + " milliseconds");
                telemetry.update();
                reverse(time);
            } else if (action == 3) {
                telemetry.addLine("Running action 'Rotate' with direction" + direction + " for " + time + " milliseconds");
                telemetry.update();
                rotate(time, direction);
            } else if (action == 4) {
                telemetry.addLine("Running action 'Strafe' with direction " + direction + " for " + time + " milliseconds");
                telemetry.update();
                strafe(time, direction);
            }
            requestOpModeStop();
        }
        try {
            sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
