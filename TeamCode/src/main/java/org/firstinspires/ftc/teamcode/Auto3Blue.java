package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDriveTrain;

@Autonomous(name = "Auto3B", group = "Blue Auto")
public class Auto3Blue extends OpMode {
    int step1 = 3000;
    int step2 = 500;
    int step3 = 1000;
    int step4 = 1000;
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
        telemetry.addLine("Blue alliance");
        telemetry.addLine("Move forward, rotate, move forward again, shoot, then strafe off line");
        telemetry.addLine("Place robot on small triangle, facing forward");
        telemetry.update();
    }


    @Override
    public void start() {
        forward(step1);
        rotate(step2, -1);
        forward(step3);
        launch.run(true);
        strafe(step4, -1);
    }
    @Override
    public void loop() {

    }
}
