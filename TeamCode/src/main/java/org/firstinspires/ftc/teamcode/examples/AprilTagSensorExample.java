package org.firstinspires.ftc.teamcode.examples;

import static org.firstinspires.ftc.teamcode.utils.Config.*;
import static org.firstinspires.ftc.teamcode.utils.Constants.*;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagSensorExample {
    private AprilTagProcessor processor;
    private AprilTagDetection currentTag;
    private List<AprilTagDetection> tags;
    private int alliance = -1;

    public void init(HardwareMap hardwareMap) {
        AprilTagProcessor.Builder processorBuilder;
        VisionPortal.Builder visionBuilder;
        VisionPortal portal;
        processorBuilder = new AprilTagProcessor.Builder();
        processor = processorBuilder.build();
        visionBuilder = new VisionPortal.Builder();
        visionBuilder.setCamera(hardwareMap.get(WebcamName.class, WEBCAM));
        visionBuilder.addProcessor(processor);
        portal = visionBuilder.build();
    }
public void setAlliance(int input_alliance){
        alliance = input_alliance;
}

public void setAllianceBlue(){
        alliance = BLUE_ALLIANCE;
}

public void setAllianceRed(){
        alliance = RED_ALLIANCE;
}
    public void run() {
        tags = processor.getDetections();
        if (!tags.isEmpty()) {
            for (AprilTagDetection t : tags) {
                if (t.id == alliance) {
                    currentTag = t;
                    break;
                }
                currentTag = tags.get(0);
            }
        }
    }
    public double getYDistance() {
        if (currentTag != null && currentTag.ftcPose != null){
            return currentTag.ftcPose.y;
        }
        return 0;
    }
}
