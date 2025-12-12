package org.firstinspires.ftc.teamcode.examples;

import static org.firstinspires.ftc.teamcode.utils.Constants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LocalizationManager {
    private final PinpointExample pinpoint = new PinpointExample();
    private final AprilTagLocalizationExample tag = new AprilTagLocalizationExample();
    private Pose2D robot_position;
    private boolean april_tag_oriented = false;
    private final ElapsedTime APRIL_TAG_TIMER = new ElapsedTime();
    private final double TIME_BETWEEN_CHECKS = 4;
    private int alliance = 0;
    private double blue_alliance_heading = 90;
    private double red_alliance_heading = -90;
    public void init(HardwareMap hardwareMap){
        tag.init(hardwareMap);
        pinpoint.init(hardwareMap);
        robot_position = pinpoint.position();
    }
    public void run(){
        tag.run();
        if(!april_tag_oriented) {
            updatePinpoint();
        }
        robot_position = pinpoint.run();
        checkAprilTagTimer();
    }
    public double goalDistance(){
        return tag.getDistanceToGoal();
    }
    public void updatePinpoint(){
        Pose2D position = aprilTagPosition2D();
        if (position == null) return;
        pinpoint.position(position);
        april_tag_oriented = true;
    }
    public void updatePinpoint(boolean button){
        if (button) {
            Pose2D position = aprilTagPosition2D();
            if (position == null) return;
            pinpoint.position(position);
            april_tag_oriented = true;
        }
    }
    public double getHeading(){
        if(alliance == BLUE_ALLIANCE){
            return robot_position.getHeading(AngleUnit.DEGREES) + blue_alliance_heading;
        } else if (alliance == RED_ALLIANCE){
            return robot_position.getHeading(AngleUnit.DEGREES) + red_alliance_heading;
        }
        return robot_position.getHeading(AngleUnit.DEGREES);
    }
    public Pose2D getRobotPosition() {
        return robot_position;
    }
    public void setAlliance(int input_alliance){
        alliance = input_alliance;
    }
    private void checkAprilTagTimer(){
        if(april_tag_oriented) {
            double time = APRIL_TAG_TIMER.time();
            if (time > TIME_BETWEEN_CHECKS) {
                APRIL_TAG_TIMER.reset();
                april_tag_oriented = false;
            }
        }
    }
    public void getTelemetry(Telemetry telemetry){
        // pinpoint.getTelemetry(telemetry);
        tag.getTelemetry(telemetry);
    }
    private Pose2D aprilTagPosition2D(){
        Pose3D position = tag.position();
        if (position == null) return null;
        double x = position.getPosition().x;
        double y = position.getPosition().y;
        double heading = position.getOrientation().getYaw(AngleUnit.DEGREES);
        return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
    }
}
