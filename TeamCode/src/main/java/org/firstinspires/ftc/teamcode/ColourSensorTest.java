package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ColourSensor;
import org.firstinspires.ftc.teamcode.mechanisms.SpinSorter;

@TeleOp
public class ColourSensorTest extends OpMode {
    SpinSorter spindexer = null;
    ColourSensor colourSensor = new ColourSensor();
    public void init(){
        colourSensor.init(hardwareMap);
        spindexer = new SpinSorter(hardwareMap, 0.105, telemetry);
        spindexer.init();
    }
    public void loop(){
        colourSensor.getColour(1);
        colourSensor.getColour(2);
        colourSensor.getColour(3);
        telemetry.update();
        if(gamepad1.bWasReleased()) {
            spindexer.SpinRight(true);
        }
    }
}
