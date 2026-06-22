package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utilities.GlobalVars;
import org.firstinspires.ftc.teamcode.utilities.Team;

@Config
@TeleOp
public class RedDrive extends Init{

    public static double limiter = 1;
    public static double fps = 24;
    public static boolean launching = true;
    public static boolean peter = false;
    private double forwardPower;
    private double rotationPower;
    private double sidewaysPower;
    private double intake;
    private double launcher;
    public static double down_pos;

    public static double up_pos;

    double start = clock.seconds();
    double now = clock.seconds();

    boolean launchSequence = false;
    private double alignmentPower;

    @Override
    public void setTeam(){
        Team.set(Team.RED);
    }

    @Override
    public void extraInit(){
        mecanumDrivetrain.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherController.launcherMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherController.launcherMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    @Override
    protected void runStrategy() {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        dashboard.startCameraStream(visionPortal,fps);

        //telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        forwardPower = 0;
        rotationPower = 0;
        sidewaysPower = 0;
        intake = 0;
        launcher = 0;

        double currentFPS = fps;

        while(opModeIsActive()) {

            telemetryPacket = new TelemetryPacket();

            if (fps != currentFPS){
                FtcDashboard.getInstance().startCameraStream(visionPortal,fps);
                currentFPS = fps;
            }

            forwardPower = -gamepad1.left_stick_y;
            sidewaysPower = gamepad1.left_stick_x;
            rotationPower = gamepad1.right_stick_x;

            if (gamepad2.a){
                launcherController.gateMotor.setPower(0.5);
                intakeMotor.setPower(-0.5);
            }

            if (gamepad2.b){
                raiseKicker();
            } else if (!launchSequence) {
                lowerKicker();
            }

            intake = (gamepad2.left_trigger-gamepad2.right_trigger+gamepad1.left_trigger);


            forwardPower = forwardPower * (1-gamepad1.right_trigger*0.8);
            rotationPower = rotationPower * (1-gamepad1.right_trigger*0.8);
            sidewaysPower = sidewaysPower * (1-gamepad1.right_trigger*0.8);

            if(gamepad1.backWasPressed()){
                if(Team.get() == Team.BLUE){
                    pinpointLocalizer.setPose(new Pose2d(61,  -14.5, Math.PI));
                }else{
                    pinpointLocalizer.setPose(new Pose2d(63.5,  15.75, Math.PI));
                }
            }

            pinpointLocalizer.update();
            Pose2d robotPose = pinpointLocalizer.getPose();

            Pair<Double, Double> goalDistAndBearing = aprilTagDetector.getGoalDistAndBearing();

            forwardPower = forwardPower * limiter;
            rotationPower = rotationPower * limiter;
            sidewaysPower = sidewaysPower * limiter;

            //pinpointComputer.update();

            /*
            telemetry.addData("pinX",pinpointComputer.getPosX(DistanceUnit.CM));
            telemetry.addData("pinY",pinpointComputer.getPosY(DistanceUnit.CM));
            telemetry.addData("pinH",pinpointComputer.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Pose2D", pinpointComputer.getPosition());
            telemetry.addData("power",power);
            telemetry.addData("direction",direction);
            telemetry.addData("sideways",sideways);
            telemetry.addData("intake", intake);
            telemetry.addData("launcher",launcherMotor.getVelocity());
            telemetry.addData("limiter",limiter);

            telemetry.addData("Y", imu.getRobotYawPitchRollAngles().getYaw());
            telemetry.addData("P", imu.getRobotYawPitchRollAngles().getPitch());
            telemetry.addData("R", imu.getRobotYawPitchRollAngles().getRoll());

             */

            intakeMotor.setPower(intake);

            launcherController.gateMotor.setPower(gamepad2.left_stick_y);

            if(gamepad1.bWasPressed()){
                goalAlignmentPID.reset(0);
            }

            if (gamepad1.b) {
                alignmentPower = alignToGoal().first;
            } else {
                alignmentPower = 0d;
            }

            if (gamepad2.x){
                launcherController.setVelocity(200);
            }

            if (gamepad2.left_bumper){
                launcherController.setVelocity(GlobalVars.defaultLauncherSpeed);
            }
            if (gamepad2.dpad_down){
                //   gateServo.setPosition(down_pos);
                //    kickerServo.setPosition(0.2);
            }
            if (gamepad2.dpad_up){
                //    gateServo.setPosition(up_pos);
                //    kickerServo.setPosition(0.07);
            }
            /*
            if (gamepad1.aWasPressed()){
                pinpointComputer.resetPosAndIMU();
            }
            */

            if (gamepad1.left_bumper) {
                mecanumDrivetrain.setOrthoAbs(sidewaysPower, forwardPower, rotationPower+alignmentPower, pinpointComputer.getHeading(AngleUnit.DEGREES));
            } else {
                mecanumDrivetrain.setOrtho(sidewaysPower, forwardPower, rotationPower+alignmentPower);
            }

            if (gamepad2.right_bumper){
                double goalDist = getGoalDist();
                //telemetry.addData("goalDist", goalDist);
                if (goalDist<=380) {
                    launchAtDist(goalDist);
                }else{
                    launcherController.setVelocity(0);
                }
            }
            /*
            else {
                launcherController.setVelocity(GlobalVars.defaultLauncherSpeed);
            }
             */

            if (gamepad2.y && !launchSequence) {
                launchSequence = true;
                start = clock.seconds();
            }
            if (launchSequence){
                now = clock.seconds()-start;

                if (now>=3){
                    gateMotor.setPower(0);
                    intakeMotor.setPower(0);
                    launchSequence = false;
                } else if (now>=1.2) {
                    gateMotor.setPower(-1);
                    intakeMotor.setPower(1);
                    mediumKicker();
                } else if (now >= 1) {
                    mediumKicker();
                    gateMotor.setPower(-1);
                }else{
                    raiseKicker();
                    gateMotor.setPower(0.3);
                }
            }
            telemetry.addData("Robot X (in)", robotPose.position.x);
            telemetry.addData("Robot Y (in)", robotPose.position.y);
            telemetry.addData("Encoder X (ticks)", pinpointComputer.getEncoderX());
            telemetry.addData("Encoder Y (ticks)", pinpointComputer.getEncoderY());
            //telemetry.addData("currentHeading", Math.toDegrees(robotPose.heading.toDouble()));

            Canvas c = telemetryPacket.fieldOverlay();

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, robotPose);

            dashboard.sendTelemetryPacket(telemetryPacket);

            telemetry.update();
            pause(0.02);
        }
    }
}
