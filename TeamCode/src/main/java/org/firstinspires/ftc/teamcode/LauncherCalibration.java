package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class LauncherCalibration extends Init{
    int setVelocity = 0;
    @Override
    protected void runStrategy() {
        FtcDashboard.getInstance().startCameraStream(visionPortal,0);
        gate.setPosition(0.4);
        while(opModeIsActive()){
            double power = gamepad1.left_stick_y;
            double direction = gamepad1.right_stick_x;
            if(gamepad1.rightBumperWasPressed()){
                setVelocity += 20;
            }
            if(gamepad1.leftBumperWasPressed()){
                setVelocity -= 20;
            }
            if(gamepad1.dpadUpWasPressed()){
                setVelocity += 200;
            }
            if(gamepad1.dpadDownWasPressed()){
                setVelocity -= 200;
            }

            if (gamepad1.a){
                launcherMotor.setVelocity(setVelocity);
            }
            else {
                launcherMotor.setVelocity(0);
            }

            launcherController.launcherMotor.setPower(-gamepad1.left_trigger);
            mecanumDrivetrain.setOrtho(power, direction);

            if(aprilTagDetector.isTagDetected(20)) {
                telemetry.addData("Distance", aprilTagDetector.getTagById(20).ftcPose.y);
            }
            telemetry.addData("Set Velocity", setVelocity);
            telemetry.addData("Current Velocity", launcherMotor.getVelocity());

            aprilTagDetector.updateTags();
            if(aprilTagDetector.isTagDetected(20)){
                AprilTagDetection goalTag = aprilTagDetector.getTagById(20);
                telemetry.addData("Bearing", goalTag.ftcPose.bearing);
                telemetry.addData("Distance", goalTag.ftcPose.y);
                if(gamepad1.bWasPressed()){
                    launchAtDist(goalTag.ftcPose.y);
                }
            }
            telemetry.update();
            pause(0.2);
        }
    }
}
