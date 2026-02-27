package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Constants.FORWARD;
import static org.firstinspires.ftc.teamcode.utils.Constants.REVERSE;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Feeder;
import org.firstinspires.ftc.teamcode.mechanisms.Flywheel;

@TeleOp
public class FlywheelLaunchTest extends OpMode {
    Flywheel flywheel = null;
    Feeder servoArm = null;
    int target = 1200;
    public void init(){
        flywheel = new Flywheel(hardwareMap, REVERSE, FORWARD, telemetry);
        flywheel.turnMotorOff(true);
        telemetry.addLine("Flywheel velocity test");
        telemetry.addLine("Flywheel will activate upon starting");
        telemetry.addLine("Use dpad to adjust target velocity in increments of 100");
        telemetry.addLine("Use A to kick feeder up");
        telemetry.update();
    }
    public void loop(){
        flywheel.turnMotorOn(true);
        telemetry.addLine("Use dpad to adjust target velocity in increments of 100");
        telemetry.addLine("Use A to kick feeder up");
        telemetry.addData("Target Velocity", target);
        telemetry.addData("Flywheel at target velocity", flywheel.is_at_target());
        telemetry.update();
        if (gamepad1.dpad_up) {
            target += 100;
        } else if (gamepad1.dpad_down) {
            target -= 100;
        }
        if (gamepad1.a) {
            servoArm.up(true);
            try {
                sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            servoArm.down(true);
        }
        try {
            sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
