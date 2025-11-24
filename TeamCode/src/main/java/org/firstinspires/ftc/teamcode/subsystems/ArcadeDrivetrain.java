package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.utils.Constants.*;

public class ArcadeDrivetrain {
private DcMotor back_left = null;
private DcMotor back_right = null;
public ArcadeDrivetrain(HardwareMap hw, int ld, int rd){
	back_left = hw.get(DcMotor.class, BACK_LEFT);
	back_right = hw.get(DcMotor.class, BACK_RIGHT);
	if (ld == FORWARD){
		back_left.setDirection(DcMotor.Direction.FORWARD);
	} else {
		back_left.setDirection(DcMotor.Direction.REVERSE);
	}
	if (rd == FORWARD){
		back_right.setDirection(DcMotor.Direction.FORWARD);
	} else {
		back_right.setDirection(DcMotor.Direction.REVERSE);
	}
}


public void drive(double forward, double rotate){

}
}
