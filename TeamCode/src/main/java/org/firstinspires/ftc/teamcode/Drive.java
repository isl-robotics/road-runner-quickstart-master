package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utilities.GlobalVars;
import org.firstinspires.ftc.teamcode.utilities.Team;

@Config
@TeleOp
public class Drive extends Init{

    public static double limiter = 1;
    public static double fps = 24;
    public static boolean launching = true;
    public static boolean peter = false;

    public static double kP,kI,kD;
    private double forwardPower;
    private double rotationPower;
    private double sidewaysPower;
    private double intake;
    private double launcher;
    public static double down_pos;

    public static double up_pos;

    @Override
    public void setTeam(){
        Team.set(Team.BLUE);
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

            TelemetryPacket packet = new TelemetryPacket();

            packet.fieldOverlay()
                    .setStrokeWidth(1)
                    .setStroke("green")
                    .setFill("red")
                    .setAlpha(1.0)
                    .strokeRect(pinpointComputer.getPosX(DistanceUnit.MM)/25.4,pinpointComputer.getPosY(DistanceUnit.MM)/25.4, 4,4 )
            ;


            dashboard.sendTelemetryPacket(packet);

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
            }else {
             //   kickerServo.setPosition(0.5);
                lowerKicker();
            }




            intake = (gamepad2.left_trigger-gamepad2.right_trigger+gamepad1.left_trigger);


            forwardPower = forwardPower * (1-gamepad1.right_trigger*0.8);
            rotationPower = rotationPower * (1-gamepad1.right_trigger*0.8);
            sidewaysPower = sidewaysPower * (1-gamepad1.right_trigger*0.8);

            Pair<Double, Double> goalDistAndBearing = aprilTagDetector.getGoalDistAndBearing();

            forwardPower = forwardPower * limiter;
            rotationPower = rotationPower * limiter;
            sidewaysPower = sidewaysPower * limiter;

            pinpointComputer.update();

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

            if(gamepad2.leftBumperWasPressed()){
                goalAlignmentPID.setPID(kP,kI,kD);
                if (goalDistAndBearing != null) {
                    goalAlignmentPID.reset(goalDistAndBearing.second);
                }
            }

            if (gamepad2.left_bumper){
              //  rotationPower = alignToGoal().first;
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
            if (gamepad1.aWasPressed()){
                pinpointComputer.resetPosAndIMU();
            }

            if (gamepad1.left_bumper) {
                mecanumDrivetrain.setOrthoAbs(sidewaysPower, forwardPower, rotationPower, pinpointComputer.getHeading(AngleUnit.DEGREES));
            } else {
                mecanumDrivetrain.setOrtho(sidewaysPower, forwardPower, rotationPower);
            }

            if (gamepad2.right_bumper && (goalDistAndBearing != null)){
                launchAtDist(goalDistAndBearing.first);
            }
            /*
            else {
                launcherController.setVelocity(GlobalVars.defaultLauncherSpeed);
            }
             */

            //telemetry.update();
            pause(0.02);
        }
    }
}
