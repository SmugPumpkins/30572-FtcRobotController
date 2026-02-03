package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class LimeLightTest extends OpMode {
    private LimeLight limelight = null;
    public void loop(){
        telemetry.addLine(Double.toString(limelight.getClosestGoalDistanceInches()));
        telemetry.update();
    }
    public LimeLightTest(){
        limelight = new LimeLight();
    }

    @Override
    public void init() {

    }
}
