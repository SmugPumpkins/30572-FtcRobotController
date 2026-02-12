package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
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
            do {
                spindexer.SpinLeft(true);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            } while(colourSensor.getColour() != GREEN);
            spindexer.SpinLeft(true);
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            launch(1, telemetry);
            spindexer.SpinLeft(true);
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            launch(2, telemetry);
            spindexer.SpinLeft(true);
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            launch(3, telemetry);
        }
        else if(motif == 22){
            do {
                spindexer.SpinLeft(true);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            } while(colourSensor.getColour() != PURPLE);
            spindexer.SpinLeft(true);
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            launch(1, telemetry);
            do {
                spindexer.SpinLeft(true);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            } while(colourSensor.getColour() != GREEN);
            spindexer.SpinLeft(true);
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            launch(2, telemetry);
            do {
                spindexer.SpinLeft(true);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            } while(colourSensor.getColour() != PURPLE);
            spindexer.SpinLeft(true);
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            launch(3, telemetry);
        }
        else if(motif == 23){
            do {
                spindexer.SpinLeft(true);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            } while(colourSensor.getColour() != PURPLE);
            spindexer.SpinLeft(true);
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            launch(1, telemetry);
            do {
                spindexer.SpinLeft(true);
                try {
                    sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();

                }
            } while(colourSensor.getColour() != PURPLE);
            spindexer.SpinLeft(true);
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            launch(2, telemetry);
            spindexer.SpinLeft(true);
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            launch(3, telemetry);
        }
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
