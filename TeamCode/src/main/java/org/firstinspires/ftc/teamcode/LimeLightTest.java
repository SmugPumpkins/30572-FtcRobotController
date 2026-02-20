package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class LimeLightTest extends OpMode {
    private Limelight3A limelight;
    private IMU imu;
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        telemetry.addLine("Initialization complete");
        telemetry.update();
    }
    public void start(){
        limelight.start();
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        Pose3D botPose = llResult.getBotpose_MT2();
        if (llResult == null && llResult.isValid()) {
            telemetry.addLine("LimeLight result detected as null");
            telemetry.addData("llResult:", llResult);
            telemetry.update();
        } else if (llResult != null && !llResult.isValid()) {
            telemetry.addLine("LimeLight result is invalid");
            telemetry.addData("llResult:", llResult);
            telemetry.update();
        } else if (llResult == null && !llResult.isValid()) {
            telemetry.addLine("LimeLight result detected as null");
            telemetry.addLine("LimeLight result is invalid");
            telemetry.addData("llResult:", llResult);
            telemetry.update();
        } else if (llResult != null && llResult.isValid()) {
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("BotPose", botPose.toString());
            telemetry.update();
        }
    }
}
