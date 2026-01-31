package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class HelloWorld extends OpMode{
    @Override
    public void init() {
        telemetry.addLine("Hello World!");
    }

    @Override
    public void loop() {
        telemetry.addLine("My name is Nathan!");
    }
}
