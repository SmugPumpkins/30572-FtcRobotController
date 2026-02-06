package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.mechanisms.ColourSensor;

@TeleOp
public class ColourSensorTest extends OpMode {
    ColourSensor colourSensor = new ColourSensor();
    public void init(){
        colourSensor.init(hardwareMap);
    }
    public void loop(){
        colourSensor.getColour(telemetry);
    }
}
