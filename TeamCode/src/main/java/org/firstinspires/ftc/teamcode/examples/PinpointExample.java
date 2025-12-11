package org.firstinspires.ftc.teamcode.examples;

import static org.firstinspires.ftc.teamcode.utils.Config.*;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PinpointExample {
    private GoBildaPinpointDriver pinpoint;
    private double x_offset = 138; // in mm
    private double y_offset = -82; // in mm
    private GoBildaPinpointDriver.EncoderDirection x_direction = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private GoBildaPinpointDriver.EncoderDirection y_direction = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public void init(HardwareMap hardwareMap){
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT);
        configurePinpoint();
        resetPinpointPosition();
    }
    public Pose2D run(){
        pinpoint.update();
        return pinpoint.getPosition();
    }
    public Pose2D position(Pose2D pose){
        pinpoint.setPosition(pose);
        return pinpoint.getPosition();
    }
    public Pose2D position(){
        return pinpoint.getPosition();
    }
    public void getTelemetry(Telemetry telemetry){
        telemetry.addLine("POSITION DATA - PINPOINT");
        telemetry.addData("Relative X", pinpoint.getPosition().getX(DistanceUnit.INCH));
        telemetry.addData("Relative Y", pinpoint.getPosition().getY(DistanceUnit.INCH));
        telemetry.addData("Heading", pinpoint.getPosition().getX(DistanceUnit.INCH));
    }
    public void resetPinpointPosition(){
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }
    private void configurePinpoint(){
        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        pinpoint.setOffsets(x_offset, y_offset, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        pinpoint.setEncoderDirections(x_direction, y_direction);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();
    }
}
