// Revised run() with switch, position epsilons and a standby timer
package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.utils.Constants.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooting {
    private enum State {
        IDLE,
        SPIN_UP,
        ACTIVATE,
        SHOOTING,
        STANDBY
    }
    private State state = State.IDLE;
    HoodControl hood = null;
    SpinSorter sorter = null;
    Feeder servoArm = null;
    Flywheel flywheel = null;
    private HardwareMap hardware_map;
    private Telemetry telemetry;

    // Standby timer
    private long standbyStartMs = 0;
    private static final long STANDBY_MS = 250;
    private static final double POS_EPS = 0.05;

    public Shooting(HardwareMap input_hardware_map, Telemetry telemetry) {
        hardware_map = input_hardware_map;
        this.telemetry = telemetry;
    }
    public void init() {
        hood = new HoodControl(hardware_map, 0.5);
        sorter = new SpinSorter(hardware_map, 0.49, telemetry);
        servoArm = new Feeder(hardware_map, telemetry);
        flywheel = new Flywheel(hardware_map, REVERSE, FORWARD, telemetry);
    }

    public void run(boolean shotPressed){
        // Run exactly one state's logic per loop using switch
        switch(state) {
            case IDLE:
                // Ensure safe defaults: servo down (rest) and flywheel off
                servoArm.down(true);      // move arm to down position
                // Do not call up() here — only command the intended action
                flywheel.turnMotorOn(false); // ensure motor is off
                if (shotPressed) {
                    state = State.SPIN_UP;
                    telemetry.addData("Shooting", "-> SPIN_UP");
                }
                break;

            case SPIN_UP:
                // Spin flywheel up
                flywheel.turnMotorOn(true);
                // Allow abort if button released before we reach speed
                if (!shotPressed) {
                    flywheel.turnMotorOn(false);
                    state = State.IDLE;
                    telemetry.addData("Shooting", "SPIN_UP aborted -> IDLE");
                } else if (flywheel.is_at_target()) {
                    state = State.ACTIVATE;
                    telemetry.addData("Shooting", "at speed -> ACTIVATE");
                }
                break;

            case ACTIVATE:
                // Move feeder/arm to deploy a ring into flywheel
                servoArm.up(true); // move arm to up/activate position
                double posA = servoArm.servoArm_position; // consider exposing a getter
                if (Math.abs(posA - 0.0) < POS_EPS) { // use epsilon instead of ==
                    state = State.SHOOTING;
                    telemetry.addData("Shooting", "-> SHOOTING");
                }
                break;

            case SHOOTING:
                // Return feeder to the down/ready position
                servoArm.down(true);
                double posS = servoArm.servoArm_position;
                if (Math.abs(posS - 0.3) < POS_EPS) {
                    // Give a small buffer before returning to IDLE
                    standbyStartMs = System.currentTimeMillis();
                    state = State.STANDBY;
                    telemetry.addData("Shooting", "-> STANDBY");
                }
                break;

            case STANDBY:
                // Wait a short time to avoid immediate transitions or bounce
                if (System.currentTimeMillis() - standbyStartMs >= STANDBY_MS) {
                    // Optionally turn off flywheel here or wait for button release
                    flywheel.turnMotorOn(false);
                    state = State.IDLE;
                    telemetry.addData("Shooting", "STANDBY timeout -> IDLE");
                }
                break;
        }

        telemetry.update();
    }
}