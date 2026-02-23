package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColourSensor {
    NormalizedColorSensor colourSensor;
    NormalizedColorSensor cs2;
    NormalizedColorSensor cs3;
    public enum colour {
        GREEN,
        PURPLE,
        NONE,
        UNKNOWN,
        INVALID
    }
    public void init(HardwareMap hardwareMap) {
        colourSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        cs2 = hardwareMap.get(NormalizedColorSensor.class, "cs2");
        cs3 = hardwareMap.get(NormalizedColorSensor.class, "cs3");
        colourSensor.setGain(16);
        cs2.setGain(16);
        cs3.setGain(16);
    }
    public colour getColour(int sensor) {
        if (sensor == 1) {
            NormalizedRGBA colors = colourSensor.getNormalizedColors(); // return 4 values
            return processColour(colors);
        } else if (sensor == 2) {
            NormalizedRGBA colors = cs2.getNormalizedColors(); // return 4 values
            return processColour(colors);
        } else if (sensor == 3) {
            NormalizedRGBA colors = cs3.getNormalizedColors(); // return 4 values
            return processColour(colors);
        }
        return colour.INVALID;
    }
    public colour processColour(NormalizedRGBA colors) {
        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;
        if (normRed < 0.3 && normGreen < 0.32 && normBlue < 0.25) {
            return colour.NONE;
        } else if (normGreen > 0.33) {
            return colour.GREEN;
        } else if (normGreen > 0.22 && normRed < 0.23 && normGreen < 0.33) {
            return colour.PURPLE;
        }
        return colour.UNKNOWN;
    }
}
