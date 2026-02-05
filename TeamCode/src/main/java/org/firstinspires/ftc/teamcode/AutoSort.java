package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.CompetitionTeleOpBlue.BALL_GREEN_DOMINANCE;
import static org.firstinspires.ftc.teamcode.CompetitionTeleOpBlue.BALL_MIN_BRIGHTNESS;
import static org.firstinspires.ftc.teamcode.CompetitionTeleOpBlue.BALL_PURPLE_MIN;
import static org.firstinspires.ftc.teamcode.utils.Constants.FORWARD;
import static org.firstinspires.ftc.teamcode.utils.Constants.REVERSE;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.mechanisms.Feeder;
import org.firstinspires.ftc.teamcode.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.SpinSorter;

public class AutoSort {
    private SpinSorter spindexer = null;
    private Flywheel flywheel = null;
    private Feeder servoArm = null;
    private ColorSensor colourSensor = null;
    private boolean isGreenBallDetected() {
        if (colourSensor == null) return false;
        int r = colourSensor.red(), g = colourSensor.green(), b = colourSensor.blue();
        int total = r + g + b;
        if (total < BALL_MIN_BRIGHTNESS) return false;
        return g > r + BALL_GREEN_DOMINANCE && g > b + BALL_GREEN_DOMINANCE;
    }
    private boolean isPurpleBallDetected() {
        if (colourSensor == null) return false;
        int r = colourSensor.red(), g = colourSensor.green(), b = colourSensor.blue();
        int total = r + g + b;
        if (total < BALL_MIN_BRIGHTNESS) return false;
        return r > BALL_PURPLE_MIN && b > BALL_PURPLE_MIN && (r + b) > g + BALL_GREEN_DOMINANCE;
    }
    public void launch(){
        flywheel.turnMotorOff(true);
        flywheel.turnMotorOn(true);
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        servoArm.up(true);
        servoArm.down(false);
        try {
            sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        servoArm.down(true);
        servoArm.up(false);
    }
    public void sort(int motif){
        if(motif == 21){
            do {
                spindexer.SpinLeft(true);
            } while(!isGreenBallDetected());
            spindexer.SpinLeft(true);
            launch();
            spindexer.SpinLeft(true);
            launch();
            spindexer.SpinLeft(true);
            launch();
        }
        else if(motif == 22){
            do {
                spindexer.SpinLeft(true);
            } while(!isPurpleBallDetected());
            spindexer.SpinLeft(true);
            launch();
            do {
                spindexer.SpinLeft(true);
            } while(!isGreenBallDetected());
            spindexer.SpinLeft(true);
            launch();
            do {
                spindexer.SpinLeft(true);
            } while(!isPurpleBallDetected());
            spindexer.SpinLeft(true);
            launch();
        }
        else if(motif == 23){
            do {
                spindexer.SpinLeft(true);
            } while(!isPurpleBallDetected());
            spindexer.SpinLeft(true);
            launch();
            do {
                spindexer.SpinLeft(true);
            } while(!isPurpleBallDetected());
            spindexer.SpinLeft(true);
            launch();
            spindexer.SpinLeft(true);
            launch();
        }
    }
    public AutoSort(){
        spindexer = new SpinSorter(hardwareMap, 0.14, telemetry);
        flywheel = new Flywheel(hardwareMap, REVERSE, FORWARD, telemetry);
        servoArm = new Feeder(hardwareMap, telemetry);
        colourSensor = hardwareMap.get(ColorSensor.class, "colourSensor");
    }
}
