package org.firstinspires.ftc.teamcode.examples;

import static org.firstinspires.ftc.teamcode.utils.Constants.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class LocalizedTeleOp extends OpMode {
    private LocalizationManager localization = new LocalizationManager();
    private MecanumExample drivetrain = new MecanumExample();
    private LaunchSystemExample launcher = new LaunchSystemExample();
    @Override
    public void init() {
        localization.init(hardwareMap);
        drivetrain.init(hardwareMap);
        drivetrain.setDriveMode(MecanumExample.DriveMode.ROBOT_RELATIVE);
        launcher.init(hardwareMap);
        localization.setAlliance(BLUE_ALLIANCE);
    }

    @Override
    public void loop() {
        localization.run();
        localization.getTelemetry(telemetry);
        drivetrain.run(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        launcher.setDistance(localization.goalDistance());
        launcher.launch(gamepad1.right_trigger > 0);
        launcher.setLauncherToIdle(gamepad2.x || gamepad1.x);
        launcher.setLauncherToReady(gamepad1.y || gamepad2.y);
    }
}
