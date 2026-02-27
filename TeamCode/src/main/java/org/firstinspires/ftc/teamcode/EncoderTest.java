package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
@Config
public class EncoderTest extends Init {
    public static double dist = 0;

    public static double kP = 0.04;
    public static double kI = 0;
    public static double kD = 0.0015;
    public static int angle = 90;

    @Override
    public void extraInit() {
        mecanumDrivetrain.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    protected void runStrategy() {
        while(opModeIsActive()){

            if(gamepad1.rightStickButtonWasPressed()){
                mecanumDrivetrainController.turnRelative(angle);
            }
            mecanumDrivetrainController.setPID(kP, kI, kD);

            pinpointComputer.update();
            telemetry.addLine("Done!");
            telemetry.addData("mot power", mecanumDrivetrainController.power);
            telemetry.addData("Target", mecanumDrivetrainController.targetAngle);
            telemetry.addData("Current", mecanumDrivetrainController.currentAngle);
            telemetry.addData("Error", mecanumDrivetrainController.errorAngle);
            telemetry.addData("imu angle", -pinpointComputer.getHeading(AngleUnit.DEGREES));
            telemetry.update();

            pause(0.03);
        }
    }
}
