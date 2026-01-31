package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.utils.Config.ServoArm;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class Feeder {
    private boolean servoArm_homed;
    public double servoArm_position;
    private final double[] positions = {0, 0.3};
    private int index = 0;

    public void up(boolean button){
        if (button) {
            index = (index + 1) % positions.length;
            servoArm_position = positions[index];
        }
        servoArm.setPosition(servoArm_position);
    }
    public void down(boolean button) {
        if (button) {
            index = (index - 1 + positions.length) % positions.length;
            servoArm_position = positions[index];
        }
        servoArm.setPosition(servoArm_position);
    }
    public boolean is_homed(){
        return servoArm_homed;
    }
    private Servo servoArm = null;
    public Feeder(HardwareMap hardware_map) {
        servoArm = hardware_map.get(Servo.class, ServoArm);
    }
}
