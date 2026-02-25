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

@Autonomous(name = "Auto4B", group = "Blue Auto")
public class Auto4Blue extends OpMode {
   int step1 = 250;
    int step2 = 500;
    int step3 = 750;
    int step4 = 750;
    int step5 = 500;
    int step6 = 3000;
    int convert = 2;
    private MecanumDriveTrain drive;
    Feeder servoArm = null;
    Flywheel flywheel = null;
    Intake intake = null;
    SpinSorter sorter = null;
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
    public void launch(){
        flywheel.turnMotorOff(true);
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
        drive = new MecanumDriveTrain(hardwareMap, telemetry);
        sorter = new SpinSorter(hardwareMap, 0.14, telemetry);
        flywheel = new Flywheel(hardwareMap, REVERSE, FORWARD, telemetry);
        servoArm = new Feeder(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        telemetry.addLine("V1");
        telemetry.addLine("Blue alliance");
        telemetry.addLine("Back up, shoot, collect balls from left, return to the line and shoot, then strafe off line");
        telemetry.addLine("Place robot facing toward goal, on the line");
        drive.init(REVERSE, FORWARD, REVERSE, FORWARD);
        intake.init(FORWARD);
        telemetry.update();
    }


    @Override
    public void start() {
        forward(step1 * convert);
        sorter.SpinRight(true);
        launch();
        sorter.SpinRight(true);
        launch();
        sorter.SpinRight(true);
        launch();
        rotate(step2 * convert, 1);
        intake.RunIntake();
        reverse(step3 * convert);
        intake.StopIntake();
        forward(step4 * convert);
        rotate(step5 * convert, 1);
        sorter.SpinRight(true);
        launch();
        sorter.SpinRight(true);
        launch();
        sorter.SpinRight(true);
        launch();
        strafe(step6 * convert, 1);
    }
    @Override
    public void loop() {

    }
}
