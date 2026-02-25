package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.utilities.GlobalVars;

public class LauncherController {
    public final DcMotorEx launcherMotor;

    public LauncherController(DcMotorEx launcherMotor){
        this.launcherMotor = launcherMotor;

        launcherMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcherMotor.setDirection(DcMotorEx.Direction.REVERSE);

        launcherMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(200,0,0,12.5));
    }

    public void launchAtDist(double tagDist){
        double speed = 0.0000324*Math.pow(tagDist, 3)-0.0147043*Math.pow(tagDist, 2)+2.0775904*tagDist+1633.0430824;
        launcherMotor.setVelocity(speed);
        while((Math.abs(launcherMotor.getVelocity()-speed)>30) && (GlobalVars.opModeIsActiveGlobal.getAsBoolean())){
            GlobalVars.pauseGlobal(0.02);
        }
        GlobalVars.pauseGlobal(0.5);
        launcherMotor.setVelocity(GlobalVars.defaultLauncherSpeed);
    }

    public void sort(){
        launcherMotor.setVelocity(500);
        while((Math.abs(launcherMotor.getVelocity()-500)>30) && (GlobalVars.opModeIsActiveGlobal.getAsBoolean())){
            GlobalVars.pauseGlobal(0.02);
        }
        GlobalVars.pauseGlobal(1);
        launcherMotor.setVelocity(GlobalVars.defaultLauncherSpeed);
    }
}
