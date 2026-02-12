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
        NONE,
        UNKNOWN
    }
    public void init(HardwareMap hardwareMap) {
        colourSensor = hardwareMap.get(NormalizedColorSensor.class, "colourSensor");
        colourSensor.setGain(16);
    }
    public colour getColour() {
        NormalizedRGBA colors = colourSensor.getNormalizedColors(); // return 4 values
        float normRed, normGreen, normBlue;
        normRed = colors.red/colors.alpha;
        normGreen = colors.green/colors.alpha;
        normBlue = colors.blue/colors.alpha;
        if(normRed < 0.3 && normGreen < 0.32 && normBlue < 0.25) {
            return colour.NONE;
        }
        else if(normGreen > 0.33) {
            return colour.GREEN;
        }
        else if(normGreen > 0.22 && normRed < 0.23 && normGreen < 0.33) {
            return colour.PURPLE;
        }
        return colour.UNKNOWN;
    }
}
