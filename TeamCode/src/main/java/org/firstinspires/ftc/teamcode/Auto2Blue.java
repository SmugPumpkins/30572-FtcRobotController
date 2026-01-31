package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDriveTrain;

@Autonomous(name = "Back up, shoot, then strafe off line", group = "Blue Auto")
public class Auto2Blue extends OpMode {
    int step1 = 1000;
    int step2 = 1000;
    private MecanumDriveTrain drive;
    private Launch_v1 launch;
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
    @Override
    public void init() {
        drive = new MecanumDriveTrain(hardwareMap);
        launch = new Launch_v1(hardwareMap);
        telemetry.addLine("V1");
        telemetry.addLine("Place robot facing toward goal, on the line");
        telemetry.update();
    }


    @Override
    public void start() {
        reverse(step1);
        launch.run(true);
        strafe(step2, -1);
    }
    @Override
    public void loop() {

    }
}
