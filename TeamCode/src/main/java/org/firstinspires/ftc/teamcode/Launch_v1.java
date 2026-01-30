//To do:
//Code detection of the flywheel's velocity, feeder and hood's angle, and sort's position
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.utils.Constants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.Feeder;
import org.firstinspires.ftc.teamcode.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.Hood;

public class Launch_v1 {
    private enum State {
        IDLE,
        SPIN_UP,
        ACTIVATE,
        SHOOTING,
        STANDBY
    }
    private int aim;
    private State state = State.IDLE;
    private Hood hood = null;
    private Feeder feed = null;
    private Flywheel flywheel = null;
    private HardwareMap hardware_map;

    public Launch_v1(HardwareMap input_hardware_map) {
        hardware_map = input_hardware_map;
    }

    public void init() {
        hood = new Hood(hardware_map, 0);
        feed = new Feeder(hardware_map);
        flywheel = new Flywheel(hardware_map, REVERSE, FORWARD);
    }

    public void run(boolean shotPressed) {
        if (state == State.IDLE) {
            hood.home();
            feed.down();
            flywheel.turnMotorOff(true);
            if (shotPressed) {
                state = State.SPIN_UP;
            }
        }
        if (state == State.SPIN_UP) {

            hood.angle(aim);
            feed.down();
            flywheel.turnMotorOn(true);
            if (flywheel.is_at_target()) {
                state = State.ACTIVATE;
            }
        }
        if (state == State.ACTIVATE) {
            feed.up();
            if (feed.is_homed()) {
                state = State.SHOOTING;
            }
        }
        if (state == State.SHOOTING) {
            feed.down();
            if (feed.is_homed()) {
                state = State.STANDBY;
            }
        }
        if (state == State.STANDBY) {
            hood.angle(aim);
            feed.down();
            flywheel.turnMotorOn(true);
            if (gamepad1.b) {
                state = State.IDLE;
            }
        }
    }
}