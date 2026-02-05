package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class AutoSortTest extends OpMode {
    private AutoSort sorter = null;
    private int input = 0;
    public void init(){
        telemetry.addLine("Autosort test");
        telemetry.addLine("Press X to sort with motif GPP");
        telemetry.addLine("Press Y to sort with motif PGP");
        telemetry.addLine("Press B to sort with motif PPG");
        telemetry.update();
        do {
            if(gamepad1.x) {
                input = 21;
            } else if(gamepad1.b) {
                input = 22;
            } else if(gamepad1.b) {
                input = 23;
            }
        } while(input == 0);
        telemetry.addLine("Variables prepped");
        telemetry.addLine("Sorting with motif " + input);
        telemetry.update();
    }
    @Override
    public void loop(){
    }
    public void run(){
        sorter.sort(input);
        telemetry.addLine("Finished");
        telemetry.update();
    }
    public AutoSortTest(){
        sorter = new AutoSort();
    }
}
