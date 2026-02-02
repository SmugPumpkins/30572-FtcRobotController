package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.utils.Config.SPINDEXER;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SpinSorter {
    private Servo spindexer = null;
    public double spindexer_position;

    public void init() {
        spindexer.setPosition(spindexer_position);
    }
    private final double[] positions = {0.05, 0.49, 0.93, 0.49};
    private int index = 0;
    public void SpinLeft(boolean button) {
        if (button) {
            index = (index + 1) % positions.length;
            spindexer_position = positions[index];
        }
        spindexer.setPosition(spindexer_position);
    }


    public void SpinRight(boolean button) {
        if (button) {
            index = (index - 1 + positions.length) % positions.length;
            spindexer_position = positions[index];
        }
        this.spindexer.setPosition(spindexer_position);
    }

    public SpinSorter(HardwareMap hardware_map, double spindexer_position) {
        this.spindexer_position = spindexer_position;
        spindexer = hardware_map.get(Servo.class, SPINDEXER);
    }

}
