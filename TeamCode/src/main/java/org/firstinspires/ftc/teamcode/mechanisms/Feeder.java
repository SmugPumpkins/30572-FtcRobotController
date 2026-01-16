package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.utils.Config.LEFT_FEEDER;
import static org.firstinspires.ftc.teamcode.utils.Config.RIGHT_FEEDER;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class Feeder {
    private boolean right_feeder_homed;
    private boolean left_feeder_homed;
    public void up(){
        right_feeder.setPosition(90);
        left_feeder.setPosition(90);
    }
    public void down(){
        right_feeder.setPosition(0);
        left_feeder.setPosition(0);
    }
    public boolean is_homed(){
        return left_feeder_homed && right_feeder_homed;
    }
    private Servo left_feeder = null;
    private Servo right_feeder = null;
    public Feeder(HardwareMap hardware_map) {
        right_feeder = hardware_map.get(Servo.class, RIGHT_FEEDER);
        left_feeder = hardware_map.get(Servo.class, LEFT_FEEDER);
    }
}
