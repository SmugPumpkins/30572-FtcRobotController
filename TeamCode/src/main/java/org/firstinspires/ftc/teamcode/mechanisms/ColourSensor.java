package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColourSensor {
    NormalizedColorSensor colourSensor;
    public enum colour {
        GREEN,
        PURPLE,
        UNKNOWN
    }
    public void init(HardwareMap hardwareMap) {
        colourSensor = hardwareMap.get(NormalizedColorSensor.class, "colourSensor");
    }
    public colour getColour(Telemetry telemetry) {
        NormalizedRGBA colors = colourSensor.getNormalizedColors(); // return 4 values
        float normRed, normGreen, normBlue;
        normRed = colors.red/colors.alpha;
        normGreen = colors.green/colors.alpha;
        normBlue = colors.blue/colors.alpha;
        telemetry.addData("Red", normRed);
        telemetry.addData("Green", normGreen);
        telemetry.addData("Blue", normBlue);
        return colour.UNKNOWN;
    }
}
