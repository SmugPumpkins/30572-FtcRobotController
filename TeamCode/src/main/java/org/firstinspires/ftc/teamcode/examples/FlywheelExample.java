package org.firstinspires.ftc.teamcode.examples;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.utils.Constants.*;
import static org.firstinspires.ftc.teamcode.utils.Config.*;

public class FlywheelExample {


    private DcMotorEx launcher;
    private int velocity_difference = 50;
    private int launcher_direction = FORWARD;
    private int min_velocity = DEFAULT_VELOCITY - velocity_difference;
    private int target_velocity = DEFAULT_VELOCITY;
    private boolean launcherActive = false;


    public void init(HardwareMap hardwareMap){
        launcher = hardwareMap.get(DcMotorEx.class, LAUNCHER);
        if (launcher_direction == FORWARD){
            launcher.setDirection(DcMotorEx.Direction.FORWARD);
        } else {
            launcher.setDirection(DcMotorEx.Direction.REVERSE);
        }
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void run(){
        if (launcherActive){
            launcher.setVelocity(target_velocity);
        } else {
            launcher.setVelocity(0);
        }
    }
    public void turnMotorOn(boolean button) {
        if (button) {
            launcherActive = true;
        }
    }

    public void turnMotorOn(){
        launcherActive = true;
    }
    public void turnMotorOff(boolean button) {
        if (button) {
            launcherActive = false;
        }
    }
    public void turnMotorOff() {
            launcherActive = false;
    }
    public boolean minVelocityReached(){
        if (launcher.getVelocity() >= min_velocity){
            return true;
        }
        return false;
    }
    public void setVelocity(int velocity) {
        min_velocity = velocity - velocity_difference;
        target_velocity = velocity;
    }

}
