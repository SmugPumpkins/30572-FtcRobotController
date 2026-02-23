package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.mechanisms.ColourSensor.colour.GREEN;
import static org.firstinspires.ftc.teamcode.mechanisms.ColourSensor.colour.PURPLE;
import static org.firstinspires.ftc.teamcode.utils.Constants.FORWARD;
import static org.firstinspires.ftc.teamcode.utils.Constants.REVERSE;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.ColourSensor;
import org.firstinspires.ftc.teamcode.mechanisms.Feeder;
import org.firstinspires.ftc.teamcode.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.SpinSorter;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class AutoSort {
    private SpinSorter spindexer = null;
    private Flywheel flywheel = null;
    private Feeder servoArm = null;
    private ColourSensor colourSensor = null;
    public void init(HardwareMap hardwareMap) {
        spindexer = new SpinSorter(hardwareMap, 0.105, telemetry);
        servoArm = new Feeder(hardwareMap, telemetry);
        flywheel = new Flywheel(hardwareMap, REVERSE, FORWARD, telemetry);
        colourSensor = new ColourSensor();
        colourSensor.init(hardwareMap);
        spindexer.init();
    }
    public void launch(int num, Telemetry telemetry) {
        telemetry.addLine("Launch " + num);
        telemetry.update();
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    public void sort(HardwareMap hardwareMap, Telemetry telemetry, int motif){
        spindexer = new SpinSorter(hardwareMap, 0.105, telemetry);
        servoArm = new Feeder(hardwareMap, telemetry);
        flywheel = new Flywheel(hardwareMap, REVERSE, FORWARD, telemetry);
        colourSensor = new ColourSensor();
        colourSensor.init(hardwareMap);
        spindexer.init();
        if(motif == 21){
            getBalls();
        }
        else if(motif == 22){
            getBalls();
        }
        else if(motif == 23){
            getBalls();
        }
        List output;
        output = getBalls();
        telemetry.addLine(output.toString());
        telemetry.update();
    }
    public List getBalls(){
        List<String> balls = new ArrayList<>();
        for (int i = 0; i < 3; i++) {
            if (colourSensor.getColour(i + 1) == GREEN) {
                balls.add("G" + i);
            } else if (colourSensor.getColour(i + 1) == PURPLE) {
                balls.add("P" + i);
            } else {
                balls.add("X" + i);
            }
        }
        return balls;
    }
    public String getVersion(){
        return "Autosort V2.0";
    }
    public AutoSort(HardwareMap hardwareMap){
        spindexer = new SpinSorter(hardwareMap, 0.14, telemetry);
        flywheel = new Flywheel(hardwareMap, REVERSE, FORWARD, telemetry);
        servoArm = new Feeder(hardwareMap, telemetry);
        spindexer.init();
        colourSensor = new ColourSensor();
        colourSensor.init(hardwareMap);
    }
}
