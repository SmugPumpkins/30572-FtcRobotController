package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp

public class Mason extends OpMode{
    @Override
    public void init(){
        telemetry.addLine("Hello Mason");
    }

    @Override
    public void loop(){
        telemetry.addLine("The robot has started!");
    }
}
