package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static org.firstinspires.ftc.teamcode.utils.Constants.*;

import org.firstinspires.ftc.teamcode.mechanisms.ArcadeDrivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDriveTrain;

@TeleOp
public class SimpleArcadeTeleOp extends OpMode{
    MecanumDriveTrain drivetrain = null;

    @Override
    public void init(){
        drivetrain = new MecanumDriveTrain(hardwareMap);
        drivetrain.init(FORWARD, FORWARD, FORWARD, FORWARD);
    }

    @Override
    public void loop(){
        drivetrain.drive(gamepad1.left_stick_y,gamepad1.left_stick_x, gamepad1.right_stick_x);
    }
}
