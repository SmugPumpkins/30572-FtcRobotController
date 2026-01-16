package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.utils.Constants.FORWARD;
import static org.firstinspires.ftc.teamcode.utils.Constants.REVERSE;

import com.qualcomm.robotcore.hardware.Servo;

public class SpinSorter {
    private Servo spin_sorter = null;

    public void init(int spin_sorter_direction) {
        if (spin_sorter_direction == FORWARD) {
            spin_sorter.setDirection(Servo.Direction.FORWARD);
        } else {
            spin_sorter.setDirection(Servo.Direction.REVERSE);
        }
    }

}
