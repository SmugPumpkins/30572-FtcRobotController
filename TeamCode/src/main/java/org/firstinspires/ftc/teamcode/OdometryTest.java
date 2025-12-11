package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class OdometryTest extends OpMode {
    private Navigate_v2 navigate = null;
    public void init(){
        navigate = new Navigate_v2();
    }
    @Override
    public void loop() {
        navigate.setTarget_x(0);
        navigate.setTarget_y(0);
    }
}
