package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Constants.FORWARD;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDriveTrain;

public class Navigate_v1 {
    MecanumDriveTrain drivetrain = null;
    private double target_x = 0;
    private double target_y = 0;
    private int x_speed = 0;
    private int y_speed = 0;
    private CoordinateSystem coords = null;
    private HardwareMap hardware_map;
    public boolean is_at_target(){
        if (coords.x == target_x){
            return true;
        } else {
            return false;
        }
    }
    public void setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods goBILDA_4_BAR_POD);
    public void init(){
        coords = new CoordinateSystem(hardware_map);
        drivetrain = new MecanumDriveTrain(hardware_map);
        drivetrain.init(FORWARD, FORWARD, FORWARD, FORWARD);
    }
    public void setTarget_y(double input_target_y){
        target_y = input_target_y;
    }
    public void setTarget_x(double input_target_x){
        target_x = input_target_x;
    }
    public Navigate_v1(HardwareMap input_hardware_map) {
        hardware_map = input_hardware_map;
    }
    public void run(){
        if (coords.x > target_x){
            int x_speed = -1;
        } else if (coords.x < target_x){
            int x_speed = 1;
        } else{
            int x_speed = 0;
        }
        if (coords.y > target_y){
            int y_speed = -1;
        } else if (coords.y < target_y){
            int y_speed = 1;
        } else {
            int y_speed = 0;
        }
        drivetrain.drive(y_speed, x_speed, 0);
    }
}