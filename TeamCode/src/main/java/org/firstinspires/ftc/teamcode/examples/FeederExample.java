package org.firstinspires.ftc.teamcode.examples;

import static org.firstinspires.ftc.teamcode.utils.Config.*;
import static org.firstinspires.ftc.teamcode.utils.Constants.*;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FeederExample {
    private CRServo left_feeder;
    private CRServo right_feeder;
    private int right_direction = FORWARD;
    private int left_direction = REVERSE;
    private ElapsedTime feeder_timer = new ElapsedTime();
    private double feed_time = 0.2;
    private boolean feed_complete = false;

    public void init(HardwareMap hardware_map){
        right_feeder = hardware_map.get(CRServo.class, RIGHT_FEEDER);
        left_feeder = hardware_map.get(CRServo.class, LEFT_FEEDER);
        if (right_direction == FORWARD){
            right_feeder.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            right_feeder.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (left_direction == FORWARD){
            left_feeder.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            left_feeder.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }
    public void feed(){
        left_feeder.setPower(1);
        right_feeder.setPower(1);
        feeder_timer.reset();
    }

    public void stopFeedWithTimer(){
        if (feeder_timer.seconds() > feed_time){
            left_feeder.setPower(0);
            right_feeder.setPower(0);
            feed_complete = true;
        }
    }

    public void stopFeedWithCustomTime(double time){
        if (feeder_timer.seconds() > time){
            left_feeder.setPower(0);
            right_feeder.setPower(0);
        }
    }

    public boolean feedComplete(){
        if (feed_complete){
            feed_complete = false;
            return true;
        }
        return feed_complete;
    }
}
