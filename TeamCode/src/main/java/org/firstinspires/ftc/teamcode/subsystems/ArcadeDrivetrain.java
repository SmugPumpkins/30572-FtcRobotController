package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.utils.Constants.*;
import static org.firstinspires.ftc.teamcode.utils.Config.*;
import com.qualcomm.robotcore.util.Range;

public class ArcadeDrivetrain {
    private DcMotor back_left = null;
    private DcMotor back_right = null;
    public ArcadeDrivetrain(HardwareMap hardware_map, int left_motor_direction, int right_motor_direction){
        back_left = hardware_map.get(DcMotor.class, BACK_LEFT);
        back_right = hardware_map.get(DcMotor.class, BACK_RIGHT);
        if (left_motor_direction == FORWARD){
            back_left.setDirection(DcMotor.Direction.FORWARD);
        } else {
            back_left.setDirection(DcMotor.Direction.REVERSE);
        }
        if (right_motor_direction == FORWARD){
            back_right.setDirection(DcMotor.Direction.FORWARD);
        } else {
            back_right.setDirection(DcMotor.Direction.REVERSE);
        }
    }
    public void drive(double forward, double rotate){
        double forward_input = -forward;
        double leftPower = Range.clip(forward_input + rotate, -1.0, 1.0);
        double rightPower = Range.clip(forward_input - rotate, -1.0, 1.0);
        back_left.setPower(leftPower);
        back_right.setPower(rightPower);
    }

}
