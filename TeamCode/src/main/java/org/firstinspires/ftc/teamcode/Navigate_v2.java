package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.utils.Config.PINPOINT;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.examples.MecanumExample;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDriveTrain;
public class Navigate_v2 {
    private GoBildaPinpointDriver pinpoint;
    MecanumExample drivetrain =new MecanumExample();
    private double target_x = 0;
    private double target_y = 0;
    private int x_speed = 0;
    private int y_speed = 0;
    private double x_offset = 138;
    private double y_offset = -82;
    private GoBildaPinpointDriver.EncoderDirection x_direction = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private GoBildaPinpointDriver.EncoderDirection y_direction = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public void init(HardwareMap hardwareMap) {
        drivetrain.init(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT);
        configurePinpoint();
        resetPinpointPosition();
    }
    public boolean is_at_target(){
        if (pinpoint.getPosition().getX(DistanceUnit.MM) == target_x && pinpoint.getPosition().getY(DistanceUnit.MM) == target_y){
            return true;
        } else {
            return false;
        }
    }
    public void setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods goBILDA_4_BAR_POD){
    }
    public void resetPinpointPosition(){
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0));
    }
    private void configurePinpoint(){
        pinpoint.setOffsets(x_offset, y_offset, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(x_direction, y_direction);
        pinpoint.resetPosAndIMU();
    }
    public void run(){

        if (pinpoint.getPosition().getX(DistanceUnit.MM) > target_x){
            int x_speed = -1;
        } else if (pinpoint.getPosition().getX(DistanceUnit.MM) < target_x){
            int x_speed = 1;
        } else{
            int x_speed = 0;
        }
        if (pinpoint.getPosition().getY(DistanceUnit.MM) > target_y){
            int y_speed = -1;
        } else if (pinpoint.getPosition().getY(DistanceUnit.MM) < target_y){
            int y_speed = 1;
        } else {
            int y_speed = 0;
        }
        drivetrain.drive(y_speed, x_speed, 0);
        pinpoint.update();
        pinpoint.getPosition();
    }
}