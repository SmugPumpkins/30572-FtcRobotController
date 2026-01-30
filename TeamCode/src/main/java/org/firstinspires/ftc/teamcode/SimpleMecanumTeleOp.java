package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static org.firstinspires.ftc.teamcode.utils.Constants.*;

import org.firstinspires.ftc.teamcode.mechanisms.ArcadeDrivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.mechanisms.SpinSorter;
import org.firstinspires.ftc.teamcode.mechanisms.Hood;
import org.firstinspires.ftc.teamcode.Launch_v1;
import org.firstinspires.ftc.teamcode.mechanisms.Feeder;
@TeleOp
public class SimpleMecanumTeleOp extends OpMode {
    MecanumDriveTrain drivetrain = null;
    Intake intake = null;
    SpinSorter sorter = null;
    Hood hood = null;
    Launch_v1 launch = null;
    Feeder servoArm = null;

    @Override
    public void init() {
        drivetrain = new MecanumDriveTrain(hardwareMap);
        intake = new Intake(hardwareMap);
        sorter = new SpinSorter(hardwareMap, 0.5);
        hood = new Hood(hardwareMap, 0);
        launch = new Launch_v1(hardwareMap);
        servoArm = new Feeder(hardwareMap);
        drivetrain.init(REVERSE, FORWARD, REVERSE, FORWARD);
        intake.init(FORWARD);
        telemetry.addLine("V16");
    }

    @Override
    public void loop() {
        drivetrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        if (gamepad1.a) {
            drivetrain.drive(0, 0, 0);
        }
        if (gamepad1.leftBumperWasPressed()) {
            intake.RunIntake();
        }
        if (gamepad1.leftBumperWasReleased()) {
            intake.StopIntake();
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
        if (gamepad1.dpadLeftWasPressed()) {
            hood.hood_down(true);
        }
        if (gamepad1.dpadRightWasPressed()) {
            hood.hood_up(true);
        }
        if (gamepad1.dpadLeftWasReleased()) {
            hood.hood_down(false);
        }
        if (gamepad1.dpadRightWasReleased()) {
            hood.hood_up(false);
        }
        if (gamepad1.rightBumperWasPressed()) {
            launch.run(true);
        }
        if (gamepad1.rightBumperWasReleased()) {
            launch.run(false);
        }
        if (gamepad1.y) {
            servoArm.up();
        }
        if (gamepad1.a) {
            servoArm.down();
        }

        telemetry.addData("Position: ", sorter.spindexer_position);

    }
}
