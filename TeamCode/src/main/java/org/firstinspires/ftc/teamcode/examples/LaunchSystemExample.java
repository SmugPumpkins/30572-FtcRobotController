package org.firstinspires.ftc.teamcode.examples;

import static org.firstinspires.ftc.teamcode.utils.Constants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class LaunchSystemExample {
    private final FeederExample feeder = new FeederExample();
    private final FlywheelExample flywheel = new FlywheelExample();
    private AprilTagSensorExample aprilTagSensor;
    private enum State {
        IDLE,
        SPIN_UP,
        READY,
        FIRE,
        FIRING
    }

    private State state = State.IDLE;

    public void init(HardwareMap hardwareMap, AprilTagSensorExample sensor){
        aprilTagSensor = sensor;
        flywheel.init(hardwareMap);
        feeder.init(hardwareMap);
    }
    public void init(HardwareMap hardwareMap){
        aprilTagSensor = new AprilTagSensorExample();
        aprilTagSensor.init(hardwareMap);
        flywheel.init(hardwareMap);
        feeder.init(hardwareMap);
    }

    public void launch(boolean shot_requested){
        flywheel.setVelocity(targetVelocityByDistance());
        flywheel.run();
        switch (state){
            case IDLE:
                flywheel.turnMotorOff();
                if(shot_requested){
                    state = State.SPIN_UP;
                }
                break;
            case SPIN_UP:
                flywheel.turnMotorOn();
                if(flywheel.minVelocityReached()){
                    state = State.FIRE;
                }
                break;
            case READY:
                flywheel.turnMotorOn();
                if (shot_requested){
                    state = State.SPIN_UP;
                }
                break;
            case FIRE:
                feeder.feed();
                state = State.FIRING;
                break;
            case FIRING:
                feeder.stopFeedWithTimer();
                if(feeder.feedComplete()){
                    state = State.READY;
                }
                break;
        }
    }

    public void setLauncherToIdle(boolean button){
        if(button){
            state = State.IDLE;
        }
    }
    public void setLauncherToIdle(){
        state = State.IDLE;
    }
    public void setLauncherToReady(boolean button){
        if(button){
            state = State.READY;
        }
    }
    public void setLauncherToReady(){
            state = State.READY;
    }


    private int targetVelocityByDistance(){
        double yDist = aprilTagSensor.getYDistance();
        int velocity = DEFAULT_VELOCITY;
        if (yDist >=  84){
            velocity = 1500;
        } else if (yDist >= 78){
            velocity = 1450;
        } else if (yDist >= 69.9){
            velocity = 1400;
        } else if (yDist >= 56){
            velocity = 1300;
        } else if (yDist >= 41.7){
            velocity = 1250;
        }
        return velocity;
    }
}
