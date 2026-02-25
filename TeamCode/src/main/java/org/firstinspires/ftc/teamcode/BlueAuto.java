
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@Autonomous
public class BlueAuto extends Init {
    private int greenPos = 0;
    private boolean obeliskDetectionFailed = true;
        @Override
        protected void runStrategy() {
            /*
            pause(0.1);
            aprilTagDetector.updateTags();
            pause(0.1);
            for (AprilTagDetection detection : aprilTagDetector.getDetectedTags()) {
                telemetry.addData("AprilTag ID", detection.id);
            }

            if (aprilTagDetector.isTagDetected(21)) {
                greenPos = 0;
                obeliskDetectionFailed = false;
            } else if (aprilTagDetector.isTagDetected(22)) {
                greenPos = 1;
                obeliskDetectionFailed = false;
            } else if (aprilTagDetector.isTagDetected(23)) {
                greenPos = 2;
                obeliskDetectionFailed = false;
            }
            telemetry.addLine("Obelisk Detection " + (obeliskDetectionFailed ? "FAILED!" : "WORKED!"));
            telemetry.addData("Green Pos", greenPos);
            telemetry.update();

            pause(0.2);

            Pair<Double, Double> distAndBearing = aprilTagDetector.getGoalDistAndBearing();

            telemetry.addLine("looped");
            telemetry.update();
            pause(0.02);

             */
            // 1. Initial Setup
            pinpointComputer.resetPosAndIMU();

            telemetry.addData("Status", "Initialized. Waiting for start...");
            telemetry.update();

            waitForStart();

            if (opModeIsActive()) {
                // 2. Your Routine
                // Example: Move 500mm forward, 200mm right, and face 90 degrees
                driveToPosition(500, 200, Math.toRadians(90), 0.7);

                pause(0.5); // Brief rest

                // Example: Return to start
                driveToPosition(0, 0, 0, 0.5);
            }
        }

        /**
         * Navigation Method using Pinpoint + MecanumDrivetrain
         */
        public void driveToPosition(double targetX, double targetY, double targetHeading, double speed) {
            while (opModeIsActive()) {
                pinpointComputer.update();
                Pose2D currentPos = pinpointComputer.getPosition();

                // 1. Calculate Field-Relative Error
                double errorX = targetX - currentPos.getX(DistanceUnit.MM);
                double errorY = targetY - currentPos.getY(DistanceUnit.MM);

                // Normalize heading error to stay between -Pi and Pi
                double headingError = targetHeading - currentPos.getHeading(AngleUnit.RADIANS);
                while (headingError > Math.PI) headingError -= 2 * Math.PI;
                while (headingError < -Math.PI) headingError += 2 * Math.PI;

                // 2. Exit condition: Are we at the target?
                if (Math.hypot(errorX, errorY) < 15 && Math.abs(headingError) < Math.toRadians(2)) {
                    mecanumDrivetrain.setOrtho(0, 0, 0);
                    break;
                }

                // 3. Coordinate Transformation (Field to Robot)
                // This allows the robot to "strafe" correctly regardless of its rotation
                double robotHeading = currentPos.getHeading(AngleUnit.RADIANS);
                double cos = Math.cos(-robotHeading);
                double sin = Math.sin(-robotHeading);

                double robotRelativeX = errorX * cos - errorY * sin;
                double robotRelativeY = errorX * sin + errorY * cos;

                // 4. Proportional Control (P-Loop)
                // Adjust these gains if the robot is too jittery or too slow
                double Kp = 0.003;      // Error is in MM, so this scales it to motor power
                double Kp_Turn = 1.2;    // Turning gain

                double driveX = robotRelativeX * Kp;
                double driveY = robotRelativeY * Kp;
                double driveTurn = headingError * Kp_Turn;

                // 5. Cap the speed to the user-defined limit
                double max = Math.max(Math.abs(driveX), Math.max(Math.abs(driveY), Math.abs(driveTurn)));
                if (max > speed) {
                    driveX = (driveX / max) * speed;
                    driveY = (driveY / max) * speed;
                    driveTurn = (driveTurn / max) * speed;
                }

                // 6. Send to your existing drivetrain method
                // X = strafe, Y = forward/backward, Turn = rotation
                mecanumDrivetrain.setOrtho(driveX, driveY, driveTurn);

                // Telemetry for debugging
                telemetry.addData("Target X", targetX);
                telemetry.addData("Target Y", targetY);
                telemetry.addData("Current Pose", currentPos.toString());
                telemetry.update();
            }
        }
    }


