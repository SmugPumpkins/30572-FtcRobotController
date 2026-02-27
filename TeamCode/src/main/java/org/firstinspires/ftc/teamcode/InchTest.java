package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Constants.FORWARD;
import static org.firstinspires.ftc.teamcode.utils.Constants.REVERSE;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDriveTrain;

@TeleOp
public class InchTest extends LinearOpMode {

    private MecanumDriveTrain drive;
    double increment = 0;
    int movement = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDriveTrain(hardwareMap, telemetry);
        drive.init(REVERSE, FORWARD, REVERSE, FORWARD);

        telemetry.addLine("Select movement:");
        telemetry.addLine("Press A to move forward");
        telemetry.addLine("Press B to move backward");
        telemetry.addLine("Press X to strafe left");
        telemetry.addLine("Press Y to strafe right");
        telemetry.addLine("Press left bumper to rotate left");
        telemetry.addLine("Press right bumper to rotate right");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {

            if (gamepad1.a) movement = 1;
            else if (gamepad1.b) movement = 2;
            else if (gamepad1.x) movement = 3;
            else if (gamepad1.y) movement = 4;
            else if (gamepad1.left_bumper) movement = 5;
            else if (gamepad1.right_bumper) movement = 6;

            if (movement != 0) {
                telemetry.addLine("Variables prepped");
            }

            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {

            increment += 10;

            telemetry.addLine("Press A to continue with next increment");
            telemetry.addLine("Increasing movement target in increments of 10 inches");
            telemetry.addData("Current movement target", increment);
            telemetry.update();

            while (!gamepad1.a && opModeIsActive()) {
                idle();
            }

            while (gamepad1.a && opModeIsActive()) {
                idle();
            }

            if (movement == 1) forward(increment);
            else if (movement == 2) backward(increment);
            else if (movement == 3) left_strafe(increment);
            else if (movement == 4) right_strafe(increment);
            else if (movement == 5) left_rotate((int) increment);
            else if (movement == 6) right_rotate((int) increment);
        }
    }

    public void backward(double inch) throws InterruptedException {
        long time = (long) (inch * (500.0 / 44.0));
        drive.drive(1, 0, 0);
        sleep(time);
        drive.drive(0, 0, 0);
    }

    public void forward(double inch) throws InterruptedException {
        long time = (long) (inch * (500.0 / 47.0));
        drive.drive(-1, 0, 0);
        sleep(time);
        drive.drive(0, 0, 0);
    }

    public void left_rotate(int degrees) throws InterruptedException {
        long time = (long) (degrees * (500.0 / 130.0));
        drive.drive(0, 0, 1);
        sleep(time);
        drive.drive(0, 0, 0);
    }

    public void right_rotate(int degrees) throws InterruptedException {
        long time = (long) (degrees * (500.0 / 145.0));
        drive.drive(0, 0, -1);
        sleep(time);
        drive.drive(0, 0, 0);
    }

    public void left_strafe(double inch) throws InterruptedException {
        long time = (long) (inch * (500.0 / 25.0));
        drive.drive(0, 1, 0);
        sleep(time);
        drive.drive(0, 0, 0);
    }

    public void right_strafe(double inch) throws InterruptedException {
        long time = (long) (inch * (500.0 / 24.0));
        drive.drive(0, -1, 0);
        sleep(time);
        drive.drive(0, 0, 0);
    }
}