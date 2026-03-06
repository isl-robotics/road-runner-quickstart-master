package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.utilities.GlobalVars;

public class LauncherController {
    public final DcMotorEx launcherMotor1;
    public final DcMotorEx launcherMotor2;

    public LauncherController(DcMotorEx launcherMotor1, DcMotorEx launcherMotor2){
        this.launcherMotor1 = launcherMotor1;
        this.launcherMotor2 = launcherMotor2;

        launcherMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcherMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcherMotor1.setDirection(DcMotorEx.Direction.REVERSE);
        launcherMotor2.setDirection(DcMotorEx.Direction.REVERSE);

    public void launchAtDist(double tagDist){
        double speed = 0.0000324*Math.pow(tagDist, 3)-0.0147043*Math.pow(tagDist, 2)+2.0775904*tagDist+1633.0430824;
        launcherMotor.setVelocity(speed);
        while((Math.abs(launcherMotor.getVelocity()-speed)>30) && (GlobalVars.opModeIsActiveGlobal.getAsBoolean())){
            GlobalVars.pauseGlobal(0.02);
        }
        GlobalVars.pauseGlobal(0.5);
        launcherMotor.setVelocity(GlobalVars.defaultLauncherSpeed);
        launcherMotor1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(200,0,0,12.5));
        launcherMotor2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(200,0,0,12.5));
    }

    public void sort(){
        launcherMotor.setVelocity(500);
        while((Math.abs(launcherMotor.getVelocity()-500)>30) && (GlobalVars.opModeIsActiveGlobal.getAsBoolean())){
            GlobalVars.pauseGlobal(0.02);
        }
        GlobalVars.pauseGlobal(1);
        launcherMotor.setVelocity(GlobalVars.defaultLauncherSpeed);
    public void setVelocity(double angularRate){
        launcherMotor2.setVelocity(angularRate);
        launcherMotor1.setVelocity(angularRate);
    }

    public double getVelocity(){return (launcherMotor1.getVelocity() + launcherMotor2.getVelocity())/2;}
}
