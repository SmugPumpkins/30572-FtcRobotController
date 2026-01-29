package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.utils.Config.SPINDEXER;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SpinSorter {
    private Servo spindexer = null;
    private double spindexer_position;

    public void init() {
        spindexer.setPosition(spindexer_position);
    }
    public void SpinLeft(boolean button) {
        if (button) {
            spindexer_position = 0.33;
        }
        spindexer.setPosition(spindexer_position);
    }
    public void SpinRight(boolean button) {
        if (button) {
            spindexer_position = 0.66;
        }
        this.spindexer.setPosition(spindexer_position);
    }
    public SpinSorter(HardwareMap hardware_map, double spindexer_position) {
        this.spindexer_position = spindexer_position;
        spindexer = hardware_map.get(Servo.class, SPINDEXER);
    }

}
