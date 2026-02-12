package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class AutoSortTest extends OpMode {
    private AutoSort sorter = null;
    private int input = 1;
    public void init(){
        sorter = new AutoSort(hardwareMap);
        sorter.init(hardwareMap);
    }
    public void init_loop(){
        if(input == 1) {
            telemetry.addLine("Autosort test");
            telemetry.addLine("Press X to sort with motif GPP");
            telemetry.addLine("Press Y to sort with motif PGP");
            telemetry.addLine("Press B to sort with motif PPG");
            telemetry.addLine("Press A to continue without setting variables and see how that ends up");
            telemetry.update();
            if (gamepad1.xWasPressed()) {
                input = 21;
            } else if (gamepad1.yWasPressed()) {
                input = 22;
            } else if (gamepad1.bWasPressed()) {
                input = 23;
            } else if (gamepad1.aWasPressed()) {
                input = 0;
            }
        }
        if (input != 1) {
            telemetry.addLine("Variables prepped");
            telemetry.addLine("Sorting with motif " + input);
            telemetry.update();
        }
    }
    @Override
    public void loop(){
    }
    public void start(){
        sorter.sort(hardwareMap, telemetry, input);
        telemetry.addLine("Finished");
        telemetry.update();
    }
}
