//To do:
//Code detection of the flywheel's velocity, feeder and hood's angle, and sort's position
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.utils.Constants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.mechanisms.Feeder;
import org.firstinspires.ftc.teamcode.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.Hood;
import org.firstinspires.ftc.teamcode.mechanisms.Sort;

public class Launch_v1 {
    private enum State {
        IDLE,
        SPIN_UP,
        ACTIVATE,
        SHOOTING,
        STANDBY
    }

    private State state = State.IDLE;
    private Hood hood = null;
    private Feeder feed = null;
    private Flywheel flywheel = null;
    private HardwareMap hardware_map;
    private Sort sort = new Sort();

    public Launch_v1(HardwareMap input_hardware_map) {
        hardware_map = input_hardware_map;
    }

    public void init() {
        hood = new Hood();
        feed = new Feeder();
        sort = new Sort();
        flywheel = new Flywheel(hardware_map, REVERSE);
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
            hood.angle();
            feed.down();
            flywheel.turnMotorOn(true);
            sort.slot_left(gamepad1.left_bumper);
            sort.slot_right(gamepad1.right_bumper);
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
            hood.angle();
            feed.down();
            flywheel.turnMotorOn(true);
            sort.slot_left(gamepad1.left_bumper);
            sort.slot_right(gamepad1.right_bumper);
            if (gamepad1.b) {
                state = State.IDLE;
            }
        }
    }
}