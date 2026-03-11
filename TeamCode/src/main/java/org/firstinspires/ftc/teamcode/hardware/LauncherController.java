package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.utilities.GlobalVars;

public class LauncherController {
    public final DcMotorEx launcherMotor1;
    public final DcMotorEx launcherMotor2;
    private final DcMotor gateMotor;

    public LauncherController(DcMotorEx launcherMotor1, DcMotorEx launcherMotor2, DcMotor gateMotor){
        this.launcherMotor1 = launcherMotor1;
        this.launcherMotor2 = launcherMotor2;
        this.gateMotor = gateMotor; // TODO: ADD INIT

        launcherMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcherMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherMotor1.setDirection(DcMotorEx.Direction.REVERSE);
        launcherMotor2.setDirection(DcMotorEx.Direction.REVERSE);

        launcherMotor1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(200,0,0,12.5));
        launcherMotor2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(200,0,0,12.5));
    }

    public void setVelocity(double angularRate){
        launcherMotor2.setVelocity(angularRate);
        launcherMotor1.setVelocity(angularRate);
    }

    public double getVelocity(){return (launcherMotor1.getVelocity() + launcherMotor2.getVelocity())/2;}

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients pidfCoefficients) {
        launcherMotor2.setPIDFCoefficients(runMode, pidfCoefficients);
        launcherMotor1.setPIDFCoefficients(runMode, pidfCoefficients);
    }
}
