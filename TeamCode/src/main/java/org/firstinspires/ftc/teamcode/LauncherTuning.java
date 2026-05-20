package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@TeleOp
public class LauncherTuning extends Init{
    public static double highVelocity = 1600;
    public static double lowVelocity = 300;

    double curSetVelocity = highVelocity;

    public static double f = 6;
    public static double p = 350;

    public static double i = 3.5;

    public static double d = 0.5;

    public static double angle = 0;
    PIDFCoefficients pidfCoefficients;

    int stepIndex = 0;

    @Override
    public void extraInit() {
        pidfCoefficients = new PIDFCoefficients(p,i,d,f);
    }

    @Override
    protected void runStrategy() {
        while(opModeIsActive()) {
            if (gamepad1.yWasPressed()) {
                if (curSetVelocity == highVelocity) {
                    curSetVelocity = lowVelocity;
                } else {
                    curSetVelocity = highVelocity;
                }
            }

            if (gamepad1.dpad_up){
              //  angle += 0.01;
                raiseKicker();

            }

            if (gamepad1.dpad_down){
            //    angle -= 0.05;
                lowerKicker();
            }

//            kickerServo.setPosition(angle);

            pidfCoefficients = new PIDFCoefficients(p, i, d, f);
            launcherController.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

            launcherController.setVelocity(curSetVelocity);

            double curVelocity = launcherController.getVelocity();
            double error = curVelocity - curSetVelocity;

            telemetry.addData("Angle", angle);
            telemetry.addData("Set velocity", curSetVelocity);
            telemetry.addData("Current velocity", curVelocity);
            telemetry.addData("Error velocity", error);
            telemetry.addLine("--------------------------------------------------");
            //telemetry.addData("P", p);
            //telemetry.addData("F", f);
            //telemetry.addData("Increment Size", stepSizes[stepIndex]);
            telemetry.update();
        }
    }
}
