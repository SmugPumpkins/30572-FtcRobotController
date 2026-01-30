package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.utils.Config.ServoArm;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class Feeder {
    private boolean servoArm_homed;
    public void up(){
        servoArm.setPosition(1);
    }
    public void down(){
        servoArm.setPosition(0);
    }
    public boolean is_homed(){
        return servoArm_homed;
    }
    private Servo servoArm = null;
    public Feeder(HardwareMap hardware_map) {
        servoArm = hardware_map.get(Servo.class, ServoArm);
    }
}
