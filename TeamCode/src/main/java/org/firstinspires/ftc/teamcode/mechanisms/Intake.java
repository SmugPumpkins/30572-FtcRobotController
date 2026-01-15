package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.utils.Config.INTAKE;
import static org.firstinspires.ftc.teamcode.utils.Constants.FORWARD;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intake = null;

    public void init(int intake_direction) {
        if (intake_direction == FORWARD) {
            intake.setDirection(DcMotor.Direction.FORWARD);
    } else {
            intake.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    public void RunIntake() {
        double power = 1.0;
        intake.setPower(power);
    }
}
