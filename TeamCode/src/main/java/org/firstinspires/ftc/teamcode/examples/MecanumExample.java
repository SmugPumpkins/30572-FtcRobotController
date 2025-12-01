package org.firstinspires.ftc.teamcode.examples;

import static org.firstinspires.ftc.teamcode.utils.Config.*;
import static org.firstinspires.ftc.teamcode.utils.Constants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumExample {
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;
    private HubIMUExample imu;
    private boolean useFieldRelativeDrive = false;
    private boolean imuSet = false;
    private int front_left_direction = FORWARD;
    private int front_right_direction = FORWARD;
    private int back_right_direction = FORWARD;
    private int back_left_direction = FORWARD;

    public void init(HardwareMap hardware_map) {
        front_left = hardware_map.get(DcMotor.class, FRONT_LEFT);
        front_right = hardware_map.get(DcMotor.class, FRONT_RIGHT);
        back_left = hardware_map.get(DcMotor.class, BACK_LEFT);
        back_right = hardware_map.get(DcMotor.class, BACK_RIGHT);
        if (front_left_direction == FORWARD) {
            front_left.setDirection(DcMotor.Direction.FORWARD);
        } else {
            front_left.setDirection((DcMotor.Direction.REVERSE));
        }
        if (front_right_direction == FORWARD) {
            front_right.setDirection(DcMotor.Direction.FORWARD);
        } else {
            front_right.setDirection((DcMotor.Direction.REVERSE));
        }
        if (back_left_direction == FORWARD) {
            back_left.setDirection(DcMotor.Direction.FORWARD);
        } else {
            back_left.setDirection((DcMotor.Direction.REVERSE));
        }
        if (back_right_direction == FORWARD) {
            back_right.setDirection(DcMotor.Direction.FORWARD);
        } else {
            back_right.setDirection((DcMotor.Direction.REVERSE));
        }
    }


    public void run(double forward, double right, double rotate) {
        if(useFieldRelativeDrive && imuSet){
            driveFieldRelative(forward, right, rotate);
        } else {
            drive(forward, right, rotate);
        }
    }

    public void drive(double forward, double right, double rotate){
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        front_left.setPower(maxSpeed * (frontLeftPower / maxPower));
        front_right.setPower(maxSpeed * (frontRightPower / maxPower));
        back_left.setPower(maxSpeed * (backLeftPower / maxPower));
        back_right.setPower(maxSpeed * (backRightPower / maxPower));
    }

    private void driveFieldRelative(double forward, double right, double rotate) {
        double input_angle = Math.atan2(forward, right);
        double magnitude = Math.hypot(right, forward);

        double heading = imu.getYaw();

        double movement_angle = AngleUnit.normalizeRadians(input_angle - heading);

        double newForward = magnitude * Math.sin(movement_angle);
        double newRight = magnitude * Math.cos(movement_angle);

        drive(newForward, newRight, rotate);
    }


    public void setRelativeDriveWithIMU(HubIMUExample input_imu){
        imu = input_imu;
        imuSet = true;
        useFieldRelativeDrive = true;
    }
}