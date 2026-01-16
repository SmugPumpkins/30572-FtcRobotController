package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.utils.Config.SPINDEXER;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Sort {
    private Servo spindexer = null;
    private int spindexer_position;
    public void slot_left(boolean button){
        spindexer_position = spindexer_position + 120;
    }
    public void slot_right(boolean button){
        spindexer_position = spindexer_position - 120;
    }
    public Sort(HardwareMap hardware_map, int spindexer_position) {
        spindexer = hardware_map.get(Servo.class, SPINDEXER);
        spindexer.setPosition(spindexer_position);
    }
}