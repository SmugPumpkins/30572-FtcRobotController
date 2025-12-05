package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ExampleTeleOp extends OpMode {
    private AprilTagSensorExample aprilTagSensor = new AprilTagSensorExample();
    private LaunchSystemExample launchSystem = new LaunchSystemExample();
    private MecanumExample mecanum = new MecanumExample();
    private HubIMUExample imu = new HubIMUExample();

    @Override
    public void init() {
        imu.init(hardwareMap);
        mecanum.init(hardwareMap);
        mecanum.setRelativeDriveWithIMU(imu);
        aprilTagSensor.init(hardwareMap);
        launchSystem.init(hardwareMap, aprilTagSensor);
    }

    @Override
    public void loop() {
        mecanum.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        aprilTagSensor.run();
        launchSystem.launch(gamepad1.right_trigger > 0);
        launchSystem.setLauncherToIdle(gamepad2.x || gamepad1.x);
        launchSystem.setLauncherToReady(gamepad1.y || gamepad2.y);
        imu.resetYaw(gamepad1.a);
    }
}
