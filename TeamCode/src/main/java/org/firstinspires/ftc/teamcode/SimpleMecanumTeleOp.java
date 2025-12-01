package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static org.firstinspires.ftc.teamcode.utils.Constants.*;

import org.firstinspires.ftc.teamcode.mechanisms.ArcadeDrivetrain;

@TeleOp
public class SimpleMecanumTeleOp extends OpMode{
    ArcadeDrivetrain drivetrain = null;

    @Override
    public void init(){
        drivetrain = new ArcadeDrivetrain(hardwareMap, REVERSE, FORWARD);
    }

    @Override
    public void loop(){
        drivetrain.drive(gamepad1.left_stick_y, gamepad1.right_stick_x);
    }
}
