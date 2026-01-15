package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static org.firstinspires.ftc.teamcode.utils.Constants.*;

import org.firstinspires.ftc.teamcode.mechanisms.ArcadeDrivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDriveTrain;

@TeleOp
public class SimpleMecanumTeleOp extends OpMode {
    MecanumDriveTrain drivetrain = null;
    Intake intake = null;


    @Override
    public void init() {
        drivetrain = new MecanumDriveTrain(hardwareMap);
        drivetrain.init(REVERSE, FORWARD, REVERSE, FORWARD);
        intake.init(FORWARD);
        telemetry.addLine("V1");
    }

    @Override
    public void loop() {
        drivetrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        if (gamepad1.a) {
            drivetrain.drive(0, 0, 0);
        }
        if (gamepad1.left_bumper) {
            intake.RunIntake();
        }
    }
}