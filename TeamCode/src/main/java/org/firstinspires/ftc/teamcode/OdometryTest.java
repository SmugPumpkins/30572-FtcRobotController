package org.firstinspires.ftc.teamcode;

public class OdometryTest {
    private Navigate_v2 navigate = null;
    public void init(){
        navigate = new Navigate_v2();
    }
    public void run(){
        navigate.setTarget_x(0);
        navigate.setTarget_y(0);
    }
}
