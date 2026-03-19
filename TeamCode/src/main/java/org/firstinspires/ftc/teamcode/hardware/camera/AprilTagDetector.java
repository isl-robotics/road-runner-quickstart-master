package org.firstinspires.ftc.teamcode.hardware.camera;

import static org.firstinspires.ftc.teamcode.utilities.GlobalVars.telemetryGlobal;

import android.util.Pair;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utilities.Team;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagDetector {
    private final AprilTagProcessor aprilTagProcessor;
    private List<AprilTagDetection> detectedTags = new ArrayList<>();


    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTagProcessor;
    }

    public AprilTagDetector() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                //.setDrawTagID(true)
                //.setDrawTagOutline(true)
                //.setDrawAxes(true)
                //.setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setLensIntrinsics(980.278506554, 980.278506554, 405.170770325, 210.517863673)
                .build();

        //aprilTagProcessor.setDecimation(2);
    }

    public void updateTags(){
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags(){
        return detectedTags;
    }

    public AprilTagDetection getTagById(int id){
        for (AprilTagDetection detection : detectedTags){
            if (detection.id == id){
                return detection;
            }
        }
        return null;
    }

    public boolean isTagDetected(int id){
        return getTagById(id) != null;
    }

    public Pair<Double, Double> getGoalDistAndBearing(){
        updateTags();

        AprilTagDetection goalTag;
        if(Team.get() == Team.BLUE){
            goalTag = getTagById(20);
        }else{
            goalTag = getTagById(24);
        }

        if (goalTag != null) {
            return new Pair<Double, Double>(goalTag.ftcPose.y, goalTag.ftcPose.bearing);
        }else{
            return null;
        }
    }

    public void detectionTelemetry(AprilTagDetection detection){
        if (detection == null){
            return;
        }
        if (detection.metadata != null) {
            telemetryGlobal.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
            telemetryGlobal.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
            telemetryGlobal.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
            telemetryGlobal.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
        } else {
            telemetryGlobal.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
            telemetryGlobal.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
        }
    }
}
