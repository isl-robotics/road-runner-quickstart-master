package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@TeleOp
public class Drive extends Init{

    public static double limiter = 1;
    public static double fps = 24;
    public static boolean launching = true;
    public static boolean peter = false;

    public static double servomax = 0.4;

    @Override
    public void extraInit(){
        mecanumDrivetrain.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    @Override
    protected void runStrategy() {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        dashboard.startCameraStream(visionPortal,fps);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        double power = 0;
        double direction = 0;
        double sideways = 0;
        double intake;
        double launcher = 0;

        imu.resetYaw();

        double currentFPS = fps;

        while(opModeIsActive()) {

            TelemetryPacket packet = new TelemetryPacket();

            packet.fieldOverlay()
                    .setStrokeWidth(1)
                    .setStroke("green")
                    .setFill("red")
                    .setAlpha(1.0)
                    .strokeRect(pinpointComputer.getPosX(DistanceUnit.MM),pinpointComputer.getPosY(DistanceUnit.MM), 4,4 )
            ;


            dashboard.sendTelemetryPacket(packet);

            if (fps != currentFPS){
                FtcDashboard.getInstance().startCameraStream(visionPortal,fps);
                currentFPS = fps;
            }

                power = -gamepad1.left_stick_y;
                sideways = gamepad1.left_stick_x;
                direction = gamepad1.right_stick_x;



            intake = -gamepad1.left_trigger;

            power = power * (1-gamepad1.right_trigger*0.8);
            direction = direction * (1-gamepad1.right_trigger*0.8);
            sideways = sideways * (1-gamepad1.right_trigger*0.8);

            if (gamepad1.a){
                intake = 1;
            }

            Pair<Double, Double> goalDistAndBearing = aprilTagDetector.getGoalDistAndBearing();

            power = power * limiter;
            direction = direction * limiter;
            sideways = sideways * limiter;

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

            if (gamepad2.right_bumper && (goalDistAndBearing != null)) {
                launchAtDist(goalDistAndBearing.first);
            }
            else {
                launcher = 0;
                launcherMotor.setVelocity(launcher);
            }

            intakeMotor.setPower(intake);
/*
            if(gamepad2.bWasPressed()){
                if (goalDistAndBearing != null) {
                    goalAlignmentPID.reset(goalDistAndBearing.second);
                }
            }
*/
            //           else{
            /*
            mecanumDrivetrain.setPowers(
                    (power+sideways)+direction,
                    (power-sideways)-direction,
                    (power - sideways) + direction,
                    (power + sideways) - direction
            );
             */

            if(gamepad1.right_stick_button) {
                mecanumDrivetrain.setOrthoAbs(sideways, power, direction, pinpointComputer.getHeading(AngleUnit.DEGREES));
            }else{
                mecanumDrivetrain.setOrtho(sideways, power, direction);
            }

//            }

            if(gamepad1.b){
                gate.setPosition(servomax);
            }
            else {
                gate.setPosition(0);
            }

            telemetry.update();
            pause(0.02);
        }
    }
}
