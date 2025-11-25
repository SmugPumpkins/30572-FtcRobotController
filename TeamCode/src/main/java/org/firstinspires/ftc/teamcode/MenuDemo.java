package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.utils.Menu;

@TeleOp(name = "MenuDemo")
public class MenuDemo extends OpMode {

    private final Menu options = new Menu(telemetry,"Settings");
    String alliance = null;
    Runnable blueAlliance = () -> {
        alliance = "Blue";
    };
    Runnable redAlliance = () -> {
        alliance = "Red";
    };

    @Override
    public void init() {
        options.addSubmenu("Select Alliance");
        Menu select_alliance = options.getMenuByName("Select Alliance");
        select_alliance.addAction("Blue", blueAlliance);
        select_alliance.addAction("Red", redAlliance);
        options.updateDisplay();
    }

    @Override
    public void init_loop(){
        options.navigate(gamepad1);
        telemetry.addData("Alliance", alliance);
    }

    @Override
    public void loop() {


    }
}
