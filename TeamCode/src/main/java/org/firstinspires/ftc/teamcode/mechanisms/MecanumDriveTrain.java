package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.utils.Config.BACK_LEFT;
import static org.firstinspires.ftc.teamcode.utils.Config.BACK_RIGHT;
import static org.firstinspires.ftc.teamcode.utils.Config.FRONT_LEFT;
import static org.firstinspires.ftc.teamcode.utils.Config.FRONT_RIGHT;
import static org.firstinspires.ftc.teamcode.utils.Config.ONBOARD_IMU;
import static org.firstinspires.ftc.teamcode.utils.Constants.FORWARD;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDriveTrain {
    private DcMotor front_left = null;
    private DcMotor front_right = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;
    private Telemetry telemetry;

    IMU imu;

    public MecanumDriveTrain (HardwareMap hardware_map, Telemetry telemetry) {
        front_left = hardware_map.get(DcMotor.class, FRONT_LEFT);
        front_right = hardware_map.get(DcMotor.class, FRONT_RIGHT);
        back_left = hardware_map.get(DcMotor.class, BACK_LEFT);
        back_right = hardware_map.get(DcMotor.class, BACK_RIGHT);
        imu = hardware_map.get(IMU.class, ONBOARD_IMU);
        //This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        this.telemetry = telemetry;
    }

    public void init(int front_left_direction, int front_right_direction, int back_left_direction, int back_right_direction) {
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

    public void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + rotate + right;
        double frontRightPower = forward - rotate - right;
        double backRightPower = forward - rotate + right;
        double backLeftPower = forward + rotate - right;

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

    public void driveFieldRelative(double forward, double right, double rotate) {
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        theta = AngleUnit.normalizeRadians(theta - robotYaw);

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive(newForward, newRight, rotate);
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public double getYawDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
