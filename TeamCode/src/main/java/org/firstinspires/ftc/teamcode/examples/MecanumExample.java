package org.firstinspires.ftc.teamcode.examples;

import static org.firstinspires.ftc.teamcode.utils.Config.*;
import static org.firstinspires.ftc.teamcode.utils.Constants.*;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumExample {
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;
    private HubIMUExample imu;
    private boolean useFieldRelativeDrive = false;
    private boolean imuSet = false;
    private final DcMotor.Direction front_left_direction = DcMotor.Direction.REVERSE;
    private final DcMotor.Direction front_right_direction = DcMotor.Direction.REVERSE;
    private final DcMotor.Direction back_left_direction = DcMotor.Direction.FORWARD;
    private final DcMotor.Direction back_right_direction = DcMotor.Direction.REVERSE;
    public enum DriveMode {
        ROBOT_RELATIVE,
        FIELD_RELATIVE_IMU,
        FIELD_RELATIVE_LOCALIZATION
    }
    private DriveMode drive_mode;


    public void init(HardwareMap hardware_map) {
        front_left = hardware_map.get(DcMotor.class, FRONT_LEFT);
        front_right = hardware_map.get(DcMotor.class, FRONT_RIGHT);
        back_left = hardware_map.get(DcMotor.class, BACK_LEFT);
        back_right = hardware_map.get(DcMotor.class, BACK_RIGHT);
        front_left.setDirection(front_left_direction);
        front_right.setDirection(front_right_direction);
        back_left.setDirection(back_left_direction);
        back_right.setDirection(back_right_direction);
        drive_mode = DriveMode.ROBOT_RELATIVE;
    }


    public void run(double forward, double right, double rotate) {
        if(useFieldRelativeDrive && imuSet){
            driveFieldRelative(forward, right, rotate);
        } else {
            drive(forward, right, rotate);
        }
    }

    public void run(double forward, double right, double rotate, double heading){
        driveFieldRelative(forward, right, rotate, heading);
    }

    public void drive(double forward, double right, double rotate){
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right + rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right - rotate;

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
    private void driveFieldRelative(double forward, double right, double rotate, double heading){
        double input_angle = Math.atan2(forward, right);
        double magnitude = Math.hypot(right, forward);

        double movement_angle = AngleUnit.normalizeRadians(input_angle - heading);

        double newForward = magnitude * Math.sin(movement_angle);
        double newRight = magnitude * Math.cos(movement_angle);

        drive(newForward, newRight, rotate);
    }
    public void setRelativeDriveWithIMU(HubIMUExample input_imu){
        imu = input_imu;
        imuSet = true;
        useFieldRelativeDrive = true;
        drive_mode = DriveMode.FIELD_RELATIVE_IMU;
    }
    public void setDriveMode(DriveMode input_drive_mode){
        drive_mode = input_drive_mode;
    }
    public void getTelemetry(Telemetry telemetry){
        telemetry.addLine("CURRENT DRIVETRAIN SETTINGS:");
        telemetry.addData("FRONT LEFT", front_left_direction);
        telemetry.addData("FRONT RIGHT", front_right_direction);
        telemetry.addData("BACK LEFT", back_left_direction);
        telemetry.addData("BACK RIGHT", back_right_direction);
        telemetry.addData("DRIVE MODE", drive_mode);
    }
}