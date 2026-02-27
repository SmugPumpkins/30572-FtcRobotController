package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.utils.Config.SPINDEXER;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class SpindexerTest extends OpMode {
    private Servo spindexer = null;
    private double pos = 0;
    public void init(){
        spindexer = hardwareMap.get(Servo.class, SPINDEXER);
        telemetry.addLine("Spindexer test");
        telemetry.addLine("Use A and B to adjust target position by increments of 0.005");
        telemetry.update();
    }
    public void loop(){
        telemetry.addLine("Spindexer test");
        telemetry.addLine("Use A and B to adjust target position by increments of 0.005");
        telemetry.addData("Target position", pos);
        telemetry.addData("Measured position", spindexer.getPosition());
        telemetry.update();
        if (gamepad1.aWasReleased()) {
            pos += 0.005;
        } else if (gamepad1.bWasReleased()) {
            pos -= 0.005;
        }
        spindexer.setPosition(pos);
        try {
            sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
