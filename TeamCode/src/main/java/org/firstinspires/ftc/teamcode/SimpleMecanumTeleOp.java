package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static org.firstinspires.ftc.teamcode.utils.Constants.*;

import org.firstinspires.ftc.teamcode.mechanisms.ArcadeDrivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.mechanisms.SpinSorter;
//import org.firstinspires.ftc.teamcode.mechanisms.Hood;
import org.firstinspires.ftc.teamcode.Launch_v1;
import org.firstinspires.ftc.teamcode.mechanisms.Feeder;
import org.firstinspires.ftc.teamcode.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.HoodControl;


@TeleOp(name="Simple Mecanum TeleOp", group ="Competition")
public class SimpleMecanumTeleOp extends OpMode {
    MecanumDriveTrain drivetrain = null;
    Intake intake = null;
    SpinSorter sorter = null;
    //Hood hood = null;
    Launch_v1 launch = null;
    Feeder servoArm = null;
    Flywheel flywheel = null;
    HoodControl hood = null;


    @Override
    public void init() {
        drivetrain = new MecanumDriveTrain(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        sorter = new SpinSorter(hardwareMap, 0.5, telemetry);
        //hood = new Hood(hardwareMap, 0);
        launch = new Launch_v1(hardwareMap ,telemetry);
        servoArm = new Feeder(hardwareMap, telemetry);
        flywheel = new Flywheel(hardwareMap, REVERSE, FORWARD, telemetry);
        drivetrain.init(REVERSE, FORWARD, REVERSE, FORWARD);
        intake.init(FORWARD);
        drivetrain.driveFieldRelative(0, 0, 0);
        telemetry.addLine("V48");
    }

    @Override
    public void loop() {
        double forward = gamepad1.left_stick_y;
        double right = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;
        drivetrain.driveFieldRelative(forward, right, rotate);
        if (gamepad1.a) {
            drivetrain.resetYaw();
        }
        if (gamepad1.leftBumperWasPressed()) {
            intake.RunIntake();
        }
        if (gamepad1.leftBumperWasReleased()) {
            intake.StopIntake();
        }
        if (gamepad2.dpadDownWasPressed()) {
            sorter.SpinLeft(true);
        }
        if (gamepad2.dpadUpWasPressed()) {
            sorter.SpinRight(true);
        }
        if (gamepad2.dpadDownWasReleased()) {
            sorter.SpinLeft(false);
            sorter.init();
        }
        if (gamepad2.dpadUpWasReleased()) {
            sorter.SpinRight(false);
            sorter.init();
        }
        if (gamepad1.dpadDownWasPressed()) {
            sorter.SpinLeft(true);
        }
        if (gamepad1.dpadUpWasPressed()) {
            sorter.SpinRight(true);
        }
        if (gamepad1.dpadDownWasReleased()) {
            sorter.SpinLeft(false);
            sorter.init();
        }
        if (gamepad1.dpadUpWasReleased()) {
            sorter.SpinRight(false);
            sorter.init();
        }
        //if (gamepad1.dpadLeftWasPressed()) {
            //hood.hood_down(true);
        //}
        //if (gamepad2.dpadRightWasPressed()) {
            //hood.hood_up(true);
        ///}
        //if (gamepad2.dpadLeftWasReleased()) {
            //hood.hood_down(false);
        //}
        //if (gamepad2.dpadRightWasReleased()) {
            //hood.hood_up(false);
        //}
        //if (gamepad2.rightBumperWasPressed()) {
            //launch.run(true);
        //}
        //if (gamepad1.rightBumperWasReleased()) {
            //launch.run(false);
        //}
        if (gamepad2.yWasPressed()) {
            servoArm.up(true);
            servoArm.down(false);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            servoArm.up(false);
            servoArm.down(true);
        }
        if (gamepad2.rightBumperWasPressed()) {
            flywheel.turnMotorOff(false);
            flywheel.turnMotorOn(true);
        }
        if (gamepad2.leftBumperWasPressed()) {
            flywheel.turnMotorOn(false);
            flywheel.turnMotorOff(true);
        }

        telemetry.addData("Position: ", sorter.spindexer_position);
        telemetry.addData("Target Velocity: ", flywheel.target_velocity);
        telemetry.addData("Minimum Velocity: ", flywheel.min_velocity);
        telemetry.addData("Velocity of Flywheel Motor 1: ", flywheel.launcher.getVelocity());
        telemetry.addData("Velocity of Flywheel Motor 2: ", flywheel.launcherTwo.getVelocity());
        //telemetry.addData("Hood Position: ", hood.hood_position);
        flywheel.run();

    }
}
