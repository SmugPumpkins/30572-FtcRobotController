package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class navigateTest {
    private Navigate_v1 navigation = null;
    private CoordinateSystem coords = null;
    private double central = 1828.75;
    private double max = 3657.5;

    public void loop() {
        telemetry.addData("X", coords.x);
        telemetry.addData("Y", coords.y);
        if (gamepad1.a) {
            navigation.setTarget_x(central);
            navigation.setTarget_y(central);
        } else if (gamepad1.dpad_down) {
            navigation.setTarget_x(central);
            navigation.setTarget_y(0);
        } else if (gamepad1.dpad_up) {
            navigation.setTarget_x(central);
            navigation.setTarget_y(max);
        } else if (gamepad1.dpad_left) {
            navigation.setTarget_x(0);
            navigation.setTarget_y(central);
        } else if (gamepad1.dpad_right) {
            navigation.setTarget_x(max);
            navigation.setTarget_y(central);
        }
    }
}