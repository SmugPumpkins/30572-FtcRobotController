package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDriveTrain;

@Autonomous(name = "Strafe off line", group= "Blue Auto")
public class Auto1Blue extends OpMode {
    int step1 = 1000;
    private MecanumDriveTrain drive;
    public void forward(long time) throws InterruptedException {
        drive.drive(1, 0, 0);
        try {
            sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        drive.drive(0, 0, 0);
    }
    public void reverse(long time) throws InterruptedException {
        drive.drive(-1, 0, 0);
        try {
            sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        drive.drive(0, 0, 0);
    }
    public void rotate(long time, int direction) throws InterruptedException {
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
        telemetry.addLine("V1");
        telemetry.addLine("Place robot on small triangle, facing forward");
        telemetry.update();
    }


    @Override
    public void start() {
        strafe(step1, -1);
    }
    @Override
    public void loop() {

    }
}
