package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Config.shooterOne;
import static org.firstinspires.ftc.teamcode.utils.Constants.FORWARD;
import static org.firstinspires.ftc.teamcode.utils.Constants.REVERSE;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.mechanisms.Feeder;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.mechanisms.SpinSorter;

@Autonomous(name = "Auto4B", group = "Blue Auto")
public class Auto4Blue extends OpMode {
    double step1 = 47.5;
    int step2 = 140;
    double step3 = 45;
    double step4 = 35;
    int step5 = 170;
    double step6 = 53;
    private MecanumDriveTrain drive;
    Feeder servoArm = null;
    public DcMotorEx launcher = null;
    Intake intake = null;
    SpinSorter sorter = null;
    public void backward(double inch, double power) {
        inch = Math.pow((-0.003767*inch), 2)+1.608894*inch + 3.765281;
        double time = (double) (inch * (500.0 / 44.0));
        drive.drive(power, 0, 0);
        try {
            sleep((long)time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        drive.drive(0, 0, 0);
    }

    public void forward(double inch, double power) {
        inch = Math.pow((-0.008891*inch), 2)+1.962775*inch + 4.442436;
        double time = (long) (inch * (500.0 / 47.0));
        drive.drive(-power, 0, 0);
        try {
            sleep((long)time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        drive.drive(0, 0, 0);
    }

    public void left_rotate(int degrees) {
        long time = (long) (degrees * (500.0 / 130.0));
        drive.drive(0, 0, 1);
        try {
            sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        drive.drive(0, 0, 0);
    }

    public void right_rotate(int degrees) {
        long time = (long) (degrees * (500.0 / 145.0));
        drive.drive(0, 0, -1);
        try {
            sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        drive.drive(0, 0, 0);
    }

    public void left_strafe(double inch) {
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

    public void right_strafe(double inch) {
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
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        servoArm.up(true);
        try {
            sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        servoArm.down(true);
        try {
            sleep(600);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    @Override
    public void init() {
        drive = new MecanumDriveTrain(hardwareMap, telemetry);
        sorter = new SpinSorter(hardwareMap, 0.1293, telemetry);
        launcher = hardwareMap.get(DcMotorEx.class, shooterOne);
        servoArm = new Feeder(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        drive.init(REVERSE, FORWARD, REVERSE, FORWARD);
        intake.init(FORWARD);
        telemetry.addLine("V1");
        telemetry.addLine("Blue alliance");
        telemetry.addLine("Back up, shoot, collect balls from left, return to the line and shoot, then strafe off line");
        telemetry.addLine("Place robot facing toward goal, on the line");
        telemetry.update();
    }


    @Override
    public void start() {
        telemetry.addLine("Autonomous in progress");
        telemetry.update();
        backward(step1, 1);
        launcher.setVelocity(1200);
        sorter.SpinRight(true);
        launch();
        sorter.SpinRight(true);
        launch();
        sorter.SpinRight(true);
        launch();
        launcher.setVelocity(0);
        right_rotate(step2);
        intake.RunIntake();
        sorter.SpinRight(true);
        backward(step3, 0.5);
        sorter.SpinLeft(true);
        backward(10, 0.5);
        sorter.SpinLeft(true);
        backward(10, 0.5);
        intake.StopIntake();
        forward(step4, 1);
        sorter.SpinRight(true);
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        sorter.SpinRight(true);
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        left_rotate(step5);
        launcher.setVelocity(1200);
        sorter.SpinRight(true);
        launch();
        sorter.SpinRight(true);
        launch();
        sorter.SpinRight(true);
        launch();
        launcher.setVelocity(0);
        right_rotate(step2);
        backward(step6, 1);
    }
    @Override
    public void loop() {
        telemetry.addLine("Autonomous finished");
        telemetry.update();
    }
}
