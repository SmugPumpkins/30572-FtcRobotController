package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.utils.Config.*;

public class Hood {

    private Servo hood = null;
    public int hood_position;
    public void home() {
        hood.setPosition(0.5);
    }
    public void angle(int position){
        hood.setPosition(position);
    }
        public Hood(HardwareMap hardware_map, int hood_position) {
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
}
