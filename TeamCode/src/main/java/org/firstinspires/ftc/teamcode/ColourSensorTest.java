package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColourSensorTest extends OpMode {
    private ColorSensor colourSensor = null;
    public void init(){
    }
    public void loop(){
        telemetry.addLine("Colour Sensor Test");
        telemetry.addData("Red: ", colourSensor.red());
        telemetry.addData("Green: ", colourSensor.green());
        telemetry.addData("Blue: ", colourSensor.blue());
        telemetry.update();
    }
    public ColourSensorTest() {
        colourSensor = hardwareMap.get(ColorSensor.class, "colourSensor");
    }
}
