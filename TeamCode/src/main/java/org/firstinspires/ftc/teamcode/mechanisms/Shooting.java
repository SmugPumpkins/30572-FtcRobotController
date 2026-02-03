package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.utils.Constants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.SpinSorter;
import org.firstinspires.ftc.teamcode.mechanisms.Feeder;
import org.firstinspires.ftc.teamcode.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.HoodControl;

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
        if (state == State.IDLE) {
            servoArm.down(true);
            servoArm.up(false);
            flywheel.turnMotorOn(false);
            flywheel.turnMotorOff(true);
            if (shotPressed) {
                state = State.SPIN_UP;
            }
        }
        if (state == State.SPIN_UP) {
            flywheel.turnMotorOff(false);
            flywheel.turnMotorOn(true);
            if (flywheel. >= flywheel.target_velocity) {
                state = State.ACTIVATE;
            }
        }
        if (state == State.ACTIVATE) {
            servoArm.down(false);
            servoArm.up(true);
            if (servoArm.servoArm_position == 0) {
                state = State.SHOOTING;
            }
        }
        if (state == State.SHOOTING) {
            servoArm.up(false);
            servoArm.down(true);
            if (servoArm.servoArm_position == 0.3) {
                state = State.STANDBY;
            }
        }
        if (state == State.STANDBY) {
            state = State.IDLE;
        }
    }
}
