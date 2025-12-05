package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Constants.FORWARD;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDriveTrain;

public class Navigate_v1 {
    MecanumDriveTrain drivetrain = null;
    private CoordinateSystem coords = null;
    private HardwareMap hardware_map;
    public void init(){
        coords = new CoordinateSystem(hardware_map);
        drivetrain = new MecanumDriveTrain(hardware_map);
        drivetrain.init(FORWARD, FORWARD, FORWARD, FORWARD);
    }
    public Navigate_v1(HardwareMap input_hardware_map) {
        hardware_map = input_hardware_map;
    }
    public void run(){
    }
}