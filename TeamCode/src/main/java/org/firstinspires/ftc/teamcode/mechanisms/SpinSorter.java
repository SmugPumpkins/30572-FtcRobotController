package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.utils.Config.SPINDEXER;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SpinSorter {
    private Telemetry telemetry;
    private Servo spindexer = null;
    public double spindexer_position;

    public void init() {
        spindexer.setPosition(spindexer_position);
    }
    private final double[] positions = {0.53, 0.14, 0.99, 0.14};
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

    public SpinSorter(HardwareMap hardware_map, double spindexer_position, Telemetry telemetry) {
        this.spindexer_position = spindexer_position;
        spindexer = hardware_map.get(Servo.class, SPINDEXER);
        this.telemetry = telemetry;
    }

}
