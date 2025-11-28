package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static org.firstinspires.ftc.teamcode.utils.Constants.*;
import org.firstinspires.ftc.teamcode.mechanisms.Flywheel;
@TeleOp
public class launcherTest extends OpMode {
    Flywheel flywheel = null;
    @Override
    public void init() {
        flywheel = new Flywheel(hardwareMap, REVERSE);
    }

    @Override
    public void loop() {
        flywheel.run();
        flywheel.turnMotorOff(gamepad1.b);
        flywheel.turnMotorOn(gamepad1.a);
    }
}
