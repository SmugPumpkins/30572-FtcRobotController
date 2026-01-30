package org.firstinspires.ftc.teamcode.mechanisms;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.utils.Constants.*;
import static org.firstinspires.ftc.teamcode.utils.Config.*;

public class Flywheel {
    public void setVelocity(int velocity) {
        min_velocity = velocity;
        target_velocity = velocity + 50;
    }


    private int min_velocity;
    private int target_velocity;

    public void turnMotorOn(boolean button) {
        if (button) {
            launcherActive = true;
        }
    }
    public void turnMotorOff(boolean button) {
        if (button) {
            launcherActive = false;
        }
    }
    public boolean is_at_target(){
        return launcher.getVelocity() >= target_velocity;
    }

    private boolean launcherActive = false;

    private DcMotorEx launcher = null;
    private DcMotorEx launcherTwo = null;
    public Flywheel(HardwareMap hardware_map, int launcher_direction, int launcherTwo_direction){
        launcher = hardware_map.get(DcMotorEx.class, shooterOne);
        launcherTwo = hardware_map.get(DcMotorEx.class, shooterTwo);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (launcher_direction == FORWARD){
            launcher.setDirection(DcMotorEx.Direction.FORWARD);
        } else {
            launcher.setDirection(DcMotorEx.Direction.REVERSE);
        } if (launcherTwo_direction == FORWARD){
            launcherTwo.setDirection(DcMotorEx.Direction.FORWARD);
        } else {
            launcherTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        min_velocity = 1350;
        target_velocity = 1400;
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void run(){
        if (launcherActive){
            launcher.setVelocity(target_velocity);
            launcherTwo.setVelocity(target_velocity);

        } else {
            launcher.setVelocity(0);
            launcherTwo.setVelocity(0);
        }
    }

}
