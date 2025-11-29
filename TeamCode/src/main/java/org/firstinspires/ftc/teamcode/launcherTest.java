package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static org.firstinspires.ftc.teamcode.utils.Constants.*;
import org.firstinspires.ftc.teamcode.mechanisms.Flywheel;
@TeleOp
@Disabled
public class launcherTest extends OpMode {
    Flywheel flywheel = null;
    Launch_v1 launcher= null;
    @Override
    public void init() {
        flywheel = new Flywheel(hardwareMap, REVERSE);
        launcher = new Launch_v1(hardwareMap);
    }

    @Override
    public void loop() {
        flywheel.run();
        flywheel.turnMotorOff(gamepad1.b);
        flywheel.turnMotorOn(gamepad1.y);
        launcher.run(gamepad1.left_bumper);
    }
}
