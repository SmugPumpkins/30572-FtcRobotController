package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Constants.FORWARD;
import static org.firstinspires.ftc.teamcode.utils.Constants.REVERSE;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Feeder;
import org.firstinspires.ftc.teamcode.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.mechanisms.SpinSorter;

@Autonomous(name = "Auto2R", group = "Red Auto")
public class Auto2Red extends OpMode {
    int step1 = 250;
    int step2 = 250;
    private MecanumDriveTrain drive;
    Feeder servoArm = null;
    Flywheel flywheel = null;
    Intake intake = null;
    SpinSorter sorter = null;
    Hood hood = null;
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
    public void launch(){
        flywheel.turnMotorOff(false);
        flywheel.turnMotorOn(true);
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        servoArm.up(true);
        servoArm.down(false);
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        servoArm.down(true);
        servoArm.up(false);
    }
    @Override
    public void init() {
        drive = new MecanumDriveTrain(hardwareMap);
        sorter = new SpinSorter(hardwareMap, 0.49);
        telemetry.addLine("V1");
        telemetry.addLine("Red alliance");
        telemetry.addLine("Back up, shoot, then strafe off line");
        telemetry.addLine("Place robot facing toward goal, on the line");
        drive.init(REVERSE, FORWARD, REVERSE, FORWARD);
        telemetry.update();
    }


    @Override
    public void start() {
        forward(step1);
        sorter.SpinLeft(true);
        launch();
        sorter.SpinRight(true);
        launch();
        sorter.SpinRight(true);
        launch();
        strafe(step2, 1);
    }
    @Override
    public void loop() {

    }
}
