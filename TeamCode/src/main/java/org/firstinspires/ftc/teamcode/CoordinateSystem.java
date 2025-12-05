//To do:
//Find if 19.894 is accurate for ticks_per_millimeter, and refine it if not
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CoordinateSystem {
    double x = 0;
    double y = 0;
    double ticks_per_millimeter = 19.894;
    private HardwareMap hardware_map;
    public CoordinateSystem(HardwareMap input_hardware_map) {
        hardware_map = input_hardware_map;
    }
    public double x(){
        return x;
    }
    public double y(){
        return y;
    }
    DcMotorEx xOdometry;
    DcMotorEx yOdometry;
     public void run(){
         xOdometry = hardware_map.get(DcMotorEx.class, "xOdometry");
         yOdometry = hardware_map.get(DcMotorEx.class, "yOdometry");
         int xTicks = xOdometry.getCurrentPosition();
         int yTicks = yOdometry.getCurrentPosition();
         double x =+ xTicks/ticks_per_millimeter;
         double y =+ yTicks/ticks_per_millimeter;
     }
}