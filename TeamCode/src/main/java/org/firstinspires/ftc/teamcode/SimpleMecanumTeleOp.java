package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static org.firstinspires.ftc.teamcode.utils.Constants.*;

import android.view.accessibility.AccessibilityNodeInfo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.ArcadeDrivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.mechanisms.SpinSorter;
import org.firstinspires.ftc.teamcode.mechanisms.Feeder;
import org.firstinspires.ftc.teamcode.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.HoodControl;
import org.firstinspires.ftc.teamcode.mechanisms.Shooting;
import org.firstinspires.ftc.teamcode.mechanisms.LimeLight_Sensor;

import java.util.List;

@TeleOp(name="Simple Mecanum TeleOp", group ="Competition")
public class SimpleMecanumTeleOp extends OpMode {
    MecanumDriveTrain drivetrain = null;
    Intake intake = null;
    SpinSorter sorter = null;
    Feeder servoArm = null;
    Flywheel flywheel = null;
    HoodControl hood = null;
    Shooting shooting = null;
    LimeLight_Sensor limelight = null;

    @Override
    public void init() {
        drivetrain = new MecanumDriveTrain(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);
        sorter = new SpinSorter(hardwareMap, 0.105, telemetry);
        hood = new HoodControl(hardwareMap, 0.5);
        servoArm = new Feeder(hardwareMap, telemetry);
        flywheel = new Flywheel(hardwareMap, REVERSE, FORWARD, telemetry);
        shooting = new Shooting(hardwareMap, telemetry);
        limelight = new LimeLight_Sensor(hardwareMap, telemetry, 0.37, 0.98, 20.0, "sightings.json");
        drivetrain.init(REVERSE, FORWARD, REVERSE, FORWARD);
        intake.init(FORWARD);
        drivetrain.driveFieldRelative(0, 0, 0);
        com.qualcomm.hardware.limelightvision.Limelight3A ll = hardwareMap.get(com.qualcomm.hardware.limelightvision.Limelight3A.class, "limelight3a");
        limelight.initLimelight(ll);
        telemetry.addLine("V61");
    }

    @Override
    public void loop() {

        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;
        drivetrain.driveFieldRelative(forward, right, rotate);
        if (gamepad1.a) {
            drivetrain.resetYaw();
        }
        if (gamepad1.leftBumperWasPressed()) {
            intake.RunIntake();
        }
        if (gamepad1.leftBumperWasReleased()) {
            intake.StopIntake();
        }
        if (gamepad2.dpadDownWasPressed()) {
            sorter.SpinLeft(true);
        }
        if (gamepad2.dpadUpWasPressed()) {
            sorter.SpinRight(true);
        }
        if (gamepad2.dpadDownWasReleased()) {
            sorter.SpinLeft(false);
            sorter.init();
        }
        if (gamepad2.dpadUpWasReleased()) {
            sorter.SpinRight(false);
            sorter.init();
        }
        if (gamepad1.dpadDownWasPressed()) {
            sorter.SpinLeft(true);
        }
        if (gamepad1.dpadUpWasPressed()) {
            sorter.SpinRight(true);
        }
        if (gamepad1.dpadDownWasReleased()) {
            sorter.SpinLeft(false);
            sorter.init();
        }
        if (gamepad1.dpadUpWasReleased()) {
            sorter.SpinRight(false);
            sorter.init();
        }
        if (gamepad2.yWasPressed()) {
            servoArm.up(true);
            servoArm.down(false);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            servoArm.up(false);
            servoArm.down(true);
        }
        if (gamepad2.rightBumperWasPressed()) {
            flywheel.turnMotorOff(false);
            flywheel.turnMotorOn(true);
        }
        if (gamepad2.leftBumperWasPressed()) {
            flywheel.turnMotorOn(false);
            flywheel.turnMotorOff(true);
        }
        if (gamepad1.dpadRightWasPressed()) {
            hood.hood_up(true);
        }
        if (gamepad1.dpadRightWasReleased()) {
            hood.hood_up(false);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            hood.hood_down(true);
        }
        if (gamepad1.dpadLeftWasReleased()) {
            hood.hood_down(false);
        }
        if (gamepad1.rightBumperWasPressed()) {
            flywheel.turnMotorOn(true);
            flywheel.turnMotorOff(false);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            servoArm.up(true);
            servoArm.down(false);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            servoArm.up(false);
            servoArm.down(true);
            sorter.SpinRight(true);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            sorter.SpinRight(false);
            servoArm.up(true);
            servoArm.down(false);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            servoArm.up(false);
            servoArm.down(true);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            sorter.SpinRight(true);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            sorter.SpinRight(false);
            servoArm.up(true);
            servoArm.down(false);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            servoArm.up(false);
            servoArm.down(true);
            flywheel.turnMotorOn(false);
            flywheel.turnMotorOff(true);

        }
        if (gamepad1.rightBumperWasReleased()) {
            flywheel.turnMotorOn(false);
            flywheel.turnMotorOff(true);
        }

        telemetry.addData("Position", sorter.spindexer_position);
        telemetry.addData("Target Velocity", flywheel.target_velocity);
        telemetry.addData("Minimum Velocity", flywheel.min_velocity);
        telemetry.addData("Velocity of Flywheel Motor 1", flywheel.launcher.getVelocity());
        telemetry.addData("Velocity of Flywheel Motor 2", flywheel.launcherTwo.getVelocity());
        telemetry.addData("Hood Position", hood.hood_position);
        telemetry.addData("Yaw (deg)", drivetrain.getYawDegrees());
        telemetry.addData("Target Distance", limelight.hasAnyGoalTarget());
        telemetry.addData("limelightEnabled", limelight != null /* or limelightEnabled if you expose it */);
        if (limelight != null) {
            com.qualcomm.hardware.limelightvision.LLResult r = limelight.getLatestResult();
            telemetry.addData("resultValid", r != null && r.isValid());
            if (r != null) {
                telemetry.addData("Ta", r.getTa());
                telemetry.addData("Fiducials count", r.getFiducialResults() == null ? 0 : r.getFiducialResults().size());
            }
        }
        telemetry.addData("limelightWrapperNull", limelight == null);
        if (limelight != null) {
            com.qualcomm.hardware.limelightvision.LLResult r = limelight.getLatestResult();
            telemetry.addData("resultNull", r == null);
            telemetry.addData("resultValid", r != null && r.isValid());
            if (r != null) {
                telemetry.addData("Ta", r.getTa());
                List<LLResultTypes.FiducialResult> fids = r.getFiducialResults();
                telemetry.addData("fiducialListNull", fids == null);
                telemetry.addData("fiducialCount", fids == null ? 0 : fids.size());
                if (fids != null) {
                    int i = 0;
                    for (com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult f : fids) {
                        telemetry.addData("fid#" + i, "id=" + f.getFiducialId());
                        i++;
                    }
                }
            }
        }
        telemetry.update();
        flywheel.run();

    }
}
