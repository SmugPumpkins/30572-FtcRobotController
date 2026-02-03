package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Constants.FORWARD;
import static org.firstinspires.ftc.teamcode.utils.Constants.REVERSE;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDriveTrain;

@Autonomous(name = "Auto1B", group= "Blue Auto")
public class Auto1Blue extends OpMode {
    int step1 = 500;
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
        drive = new MecanumDriveTrain(hardwareMap, telemetry);
        telemetry.addLine("V1");
        telemetry.addLine("Blue alliance");
        telemetry.addLine("Strafe off line");
        telemetry.addLine("Place robot on small triangle, facing forward");
        drive.init(REVERSE, FORWARD, REVERSE, FORWARD);
        telemetry.update();
    }


    @Override
    public void start() {
        strafe(step1, 1);
    }
    @Override
    public void loop() {

    }
}
