package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Constants.FORWARD;
import static org.firstinspires.ftc.teamcode.utils.Constants.REVERSE;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDriveTrain;
@TeleOp
public class InchTest extends OpMode{
    private MecanumDriveTrain drive;
    double increment = 10;
    int movement = 0;
    public void backward(double inch){
        long time = (long)inch * (500/44);
        drive.drive(1, 0, 0);
        try {
            sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        drive.drive(0, 0, 0);
    }
    public void forward(double inch){
        long time = (long)inch * (500/47);
        drive.drive(-1, 0, 0);
        try {
            sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        drive.drive(0, 0, 0);
    }
    public void left_rotate(int degrees) {
        long time = (long)degrees * (500/130);
        drive.drive(0, 0, 1);
        try {
            sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        drive.drive(0, 0, 0);
    }
    public void right_rotate(int degrees) {
        long time = (long)degrees * (500/145);
        drive.drive(0, 0, -1);
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
    public void left_strafe(double inch){
        long time = (long)inch * (500/25);
        drive.drive(0, 1, 0);
        try {
            sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        drive.drive(0, 0, 0);
    }
    public void right_strafe(double inch){
        long time = (long)inch * (500/24);
        drive.drive(0, -1, 0);
        try {
            sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        drive.drive(0, 0, 0);
    }
    @Override
    public void init(){
        drive = new MecanumDriveTrain(hardwareMap, telemetry);
        drive.init(REVERSE, FORWARD, REVERSE, FORWARD);
        telemetry.addLine("Select movement:");
        telemetry.addLine("Press A to move forward");
        telemetry.addLine("Press B to move backward");
        telemetry.addLine("Press X to strafe left");
        telemetry.addLine("Press Y to strafe right");
        telemetry.addLine("Press left bumper to rotate left");
        telemetry.addLine("Press right bumper to rotate right");
        telemetry.addLine("DO NOT CONTINUE WITHOUT SETTING THE MOVEMENT, YOU WILL GET AN ERROR");
        telemetry.addLine("You have been warned.");
        telemetry.update();
    }
    public void init_loop(){
        if (gamepad1.aWasPressed()) {
            movement = 1;
        } else if (gamepad1.bWasPressed()) {
            movement = 2;
        } else if (gamepad1.xWasPressed()) {
            movement = 3;
        } else if (gamepad1.yWasPressed()) {
            movement = 4;
        } else if (gamepad1.leftBumperWasPressed()) {
            movement = 5;
        } else if (gamepad1.rightBumperWasPressed()) {
            movement = 6;
        }
        if (movement != 0) {
            telemetry.addLine("Variables prepped");
            telemetry.update();
        }
    }
    @Override
    public void loop(){
        increment = increment + 10;
        telemetry.addLine("Press A to continue with next increment");
        telemetry.addLine("Increasing movement target in increments of 10 inches");
        telemetry.addData("Current movement target", increment);
        telemetry.update();
        while(!gamepad1.aWasPressed()){}
        if (movement == 1) {
            forward(increment);
        } else if (movement == 2) {
            backward(increment);
        } else if (movement == 3) {
            left_strafe(increment);
        } else if (movement == 4) {
            right_strafe(increment);
        } else if (movement == 5) {
            left_rotate((int)increment);
        } else if (movement == 6) {
            right_rotate((int)increment);
        }
    }
}
