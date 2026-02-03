package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.utils.Config.*;

public class HoodControl {
    private Servo hood = null;

    public double hood_position;

    public void init(int position){
        hood.setPosition(position);
    }
    public HoodControl(HardwareMap hardware_map, int hood_position) {
        hood = hardware_map.get(Servo.class, HOOD);
        hood.setPosition(hood_position);
    }
    public void hood_up(boolean button){
        if (button){
            hood_position += 0.05;
        }
        hood.setPosition(hood_position);
    }
    public void hood_down(boolean button){
        if (button){
            hood_position -= 0.05;
        }
        hood.setPosition(hood_position);
    }
    public HoodControl(HardwareMap hardware_map, double hood_position) {
        this.hood_position = hood_position;
        hood = hardware_map.get(Servo.class, HOOD);
    }
}
