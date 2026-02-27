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
    double step1 = 56.5;
    int step2 = 40;
    double step3 = 29;
    double step4 = 29;
    int step5 = 40;
    double step6 = 29;
    private MecanumDriveTrain drive;
    Feeder servoArm = null;
    Flywheel flywheel = null;
    Intake intake = null;
    SpinSorter sorter = null;
    private Launch_v1 launch;
    public void backward(double inch) throws InterruptedException {
        inch = Math.pow((-0.003767*inch), 2)+1.608894*inch + 3.765281;
        double time = (double) (inch * (500.0 / 44.0));
        drive.drive(1, 0, 0);
        try {
            sleep((long)time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        drive.drive(0, 0, 0);
    }

    public void forward(double inch) throws InterruptedException {
        inch = Math.pow((-0.008891*inch), 2)+1.962775*inch + 4.442436;
        double time = (long) (inch * (500.0 / 47.0));
        drive.drive(-1, 0, 0);
        try {
            sleep((long)time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        drive.drive(0, 0, 0);
    }

    public void left_rotate(int degrees) throws InterruptedException {
        long time = (long) (degrees * (500.0 / 130.0));
        drive.drive(0, 0, 1);
        try {
            sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        drive.drive(0, 0, 0);
    }

    public void right_rotate(int degrees) throws InterruptedException {
        long time = (long) (degrees * (500.0 / 145.0));
        drive.drive(0, 0, -1);
        try {
            sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        drive.drive(0, 0, 0);
    }

    public void left_strafe(double inch) throws InterruptedException {
        inch = Math.pow((0.002457*inch), 2)+0.886793*inch + 4.093311;
        double time = (double) (inch * (500.0 / 25.0));
        drive.drive(0, 1, 0);
        try {
            sleep((long)time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        drive.drive(0, 0, 0);
    }

    public void right_strafe(double inch) throws InterruptedException {
        inch = inch * 0.996389 + 3.47166;
        double time = (double) (inch * (500.0 / 24.0));
        drive.drive(0, -1, 0);
        try {
            sleep((long)time);
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
        backward(step1);
        sorter.SpinRight(true);
        launch();
        sorter.SpinRight(true);
        launch();
        sorter.SpinRight(true);
        launch();
        left_rotate(step2);
        intake.RunIntake();
        forward(step3);
        intake.StopIntake();
        backward(step4);
        right_rotate(step5);
        sorter.SpinRight(true);
        launch();
        sorter.SpinRight(true);
        launch();
        sorter.SpinRight(true);
        launch();
        left_strafe(step6);
    }
    @Override
    public void loop() {

    }
}
