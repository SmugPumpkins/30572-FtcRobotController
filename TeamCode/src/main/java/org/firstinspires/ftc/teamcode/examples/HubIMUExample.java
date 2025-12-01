package org.firstinspires.ftc.teamcode.examples;

import static org.firstinspires.ftc.teamcode.utils.Config.*;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class HubIMUExample {
    private IMU imu;

    // Logo direction relative to robot
    private RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    // USB direction relative to robot
    private RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    private RevHubOrientationOnRobot orientationOnRobot = new
            RevHubOrientationOnRobot(logoDirection, usbDirection);

    public void init(HardwareMap hardware_map){
        imu = hardware_map.get(IMU.class, ONBOARD_IMU);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public double getYaw(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }
    public void resetYaw(boolean button){
        if(button) {
            imu.resetYaw();
        }
    }
    public void resetYaw(){
        imu.resetYaw();
    }
}
