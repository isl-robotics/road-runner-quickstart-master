package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@TeleOp
public class LauncherTuning extends Init{
    public static double highVelocity = 2000;
    public static double lowVelocity = 950;

    double curSetVelocity = highVelocity;

    public static double f = 13.5;
    public static double p = 200;
    PIDFCoefficients pidfCoefficients;

    int stepIndex = 0;

    @Override
    public void extraInit() {
        pidfCoefficients = new PIDFCoefficients(p,0,0,f);
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
            pidfCoefficients = new PIDFCoefficients(p, 0, 0, f);
            launcherController.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

            launcherController.setVelocity(curSetVelocity);

            double curVelocity = launcherController.getVelocity();
            double error = curVelocity - curSetVelocity;

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
