package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class OdometryTest extends OpMode {
    private Navigate_v2 navigate = null;
    public void init(){
        navigate = new Navigate_v2();
        navigate.init(hardwareMap);
        navigate.setTarget_x(500);
        navigate.setTarget_y(0);
    }
    @Override
    public void loop() {
        telemetry.addData("X", navigate.x_coord());
        telemetry.addData("Y", navigate.y_coord());
        updateTelemetry(telemetry);
        navigate.run();
    }
}
