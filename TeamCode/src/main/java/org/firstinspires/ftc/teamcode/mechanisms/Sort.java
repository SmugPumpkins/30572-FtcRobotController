package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.utils.Config.SPINDEXER;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Sort {
    private Servo spindexer = null;
    private double spindexer_position;
    public void init() {
        spindexer.setPosition(spindexer_position);
    }
    public void slot_left(boolean button){
        if(button) {
           // spindexer_position = spindexer_position + 0.120;
            spindexer_position = 0.01;
        }
        this.spindexer.setPosition(spindexer_position);
    }
    public void slot_right(boolean button){
        if(button) {
            //spindexer_position = spindexer_position - 0.120;
            spindexer_position = 0.01;
        }
        this.spindexer.setPosition(spindexer_position);
    }

    public Sort(HardwareMap hardware_map, double spindexer_position) {
        this.spindexer_position = spindexer_position;
        spindexer = hardware_map.get(Servo.class, SPINDEXER);
        spindexer.setPosition(spindexer_position);
    }
}