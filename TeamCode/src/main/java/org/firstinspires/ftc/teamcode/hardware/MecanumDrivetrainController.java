package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.utilities.GlobalVars.opModeIsActiveGlobal;
import static org.firstinspires.ftc.teamcode.utilities.GlobalVars.pauseGlobal;
import static org.firstinspires.ftc.teamcode.utilities.GlobalVars.telemetryGlobal;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utilities.UniversalPID;

public class MecanumDrivetrainController {
    protected GoBildaPinpointDriver pinpointComputer;
    private final MecanumDrivetrain mecanumDrivetrain;

    public double dist = 0;
    public double theta = 0;

    public double power = 0;
    public double currentAngle = 0;
    public double targetAngle = 0;
    private UniversalPID drivePID = new UniversalPID(0.0525, 0, 0.0015);
    private UniversalPID turnPID = new UniversalPID(0.04, 0, 0.0015); //Tuned
    public double errorAngle;
    public double errorDist;

    public MecanumDrivetrainController(MecanumDrivetrain mecanumDrivetrain, GoBildaPinpointDriver pinpointComputer){
        this.mecanumDrivetrain = mecanumDrivetrain;
        this.pinpointComputer = pinpointComputer;

        drivePID.setFinishedConditions(0.5, 1.25);
        turnPID.setFinishedConditions(0.5, 2); //Tuned
    }

    public void runToAbs(double targetX, double targetY){
        pinpointComputer.update();
        targetAngle = getIMUYaw();

        currentAngle = getIMUYaw();

        double errorAngle = targetAngle-currentAngle;
        if(errorAngle > 180){
            errorAngle -= 360;
        }else if(errorAngle < -180){
            errorAngle += 360;
        }

        turnPID.reset(errorAngle);

        double curX = -pinpointComputer.getPosY(DistanceUnit.CM);
        double curY = pinpointComputer.getPosX(DistanceUnit.CM);

        double errorX = targetX-curX;
        double errorY = targetY-curY;

        errorDist = Math.hypot(errorX, errorY);

        drivePID.reset(errorDist);

        while(opModeIsActiveGlobal.getAsBoolean() && !(drivePID.isDone())){
            pinpointComputer.update();
            currentAngle = getIMUYaw();

            errorAngle = targetAngle-currentAngle;
            if(errorAngle > 180){
                errorAngle -= 360;
            }else if(errorAngle < -180){
                errorAngle += 360;
            }

            double turnPower = turnPID.compute(errorAngle);

            curX = -pinpointComputer.getPosY(DistanceUnit.CM);
            curY = pinpointComputer.getPosX(DistanceUnit.CM);

            errorX = targetX-curX;
            errorY = targetY-curY;

            errorDist = Math.hypot(errorX, errorY);
            theta = Math.toDegrees(Math.atan2(errorY,errorX));

            power = drivePID.compute(errorDist);


            mecanumDrivetrain.setPolar(power, theta-currentAngle, turnPower);

            telem();
            pauseGlobal(0.02);
        }

        turnAbsolute(targetAngle);

        mecanumDrivetrain.rotate(0);
    }

    public void turnRelative(double degrees){
        pinpointComputer.update();
        if(Math.abs(degrees)>360){
            degrees = degrees%360;
        }

        double initialAngle = getIMUYaw();

        targetAngle = initialAngle+degrees;
        if (targetAngle < -180){
            targetAngle += 360;
        }else if (targetAngle > 180){
            targetAngle -= 360;
        }

        turnAbsolute(targetAngle);
    }

    public void turnRelative(double degrees, double maxAbsSpeed){ // For slower movements
        pinpointComputer.update();
        if(Math.abs(degrees)>360){
            degrees = degrees%360;
        }

        double initialAngle = getIMUYaw();

        targetAngle = initialAngle+degrees;
        if (targetAngle < -180){
            targetAngle += 360;
        }else if (targetAngle > 180){
            targetAngle -= 360;
        }

        turnAbsolute(targetAngle, maxAbsSpeed);
    }

    public void turnAbsolute(double targetAngle){
        pinpointComputer.update();
        this.targetAngle = targetAngle;

        if (targetAngle < -180){
            targetAngle += 360;
        }else if (targetAngle > 180){
            targetAngle -= 360;
        }

        currentAngle = getIMUYaw();

        errorAngle = targetAngle - currentAngle;
        if(errorAngle > 180){
            errorAngle -= 360;
        }else if(errorAngle < -180){
            errorAngle += 360;
        }
        turnPID.reset(errorAngle);

        while(opModeIsActiveGlobal.getAsBoolean() && !(turnPID.isDone())){
            pinpointComputer.update();
            currentAngle = getIMUYaw();

            errorAngle = targetAngle-currentAngle;
            if(errorAngle > 180){
                errorAngle -= 360;
            }else if(errorAngle < -180){
                errorAngle += 360;
            }

            power = turnPID.compute(errorAngle);

            //power = Math.abs(errorAngle)>20 ? power : 0.7*power;

            mecanumDrivetrain.rotate(power);

            telem();
            pauseGlobal(0.02);
        }

        mecanumDrivetrain.rotate(0);
    }

    public void turnAbsolute(double targetAngle, double maxAbsPower){ // For slower movements
        pinpointComputer.update();
        this.targetAngle = targetAngle;

        if (targetAngle < -180){
            targetAngle += 360;
        }else if (targetAngle > 180){
            targetAngle -= 360;
        }

        currentAngle = getIMUYaw();

        errorAngle = targetAngle - currentAngle;
        if(errorAngle > 180){
            errorAngle -= 360;
        }else if(errorAngle < -180){
            errorAngle += 360;
        }
        turnPID.reset(errorAngle);

        while(opModeIsActiveGlobal.getAsBoolean() && !(turnPID.isDone())){
            pinpointComputer.update();
            currentAngle = getIMUYaw();

            errorAngle = targetAngle-currentAngle;
            if(errorAngle > 180){
                errorAngle -= 360;
            }else if(errorAngle < -180){
                errorAngle += 360;
            }

            power = turnPID.compute(errorAngle);

            power = Math.abs(errorAngle)>20 ? power : 0.7*power;

            mecanumDrivetrain.rotate(Range.clip(power, -maxAbsPower, maxAbsPower));

            telem();
            pauseGlobal(0.02);
        }

        mecanumDrivetrain.rotate(0);
    }

    private void telem(){
        telemetryGlobal.addData("mot power", power*targetAngle);
        telemetryGlobal.addData("Target", targetAngle);
        telemetryGlobal.addData("Current", currentAngle);
        telemetryGlobal.addData("Error", errorAngle);
        telemetryGlobal.addData("imu angle", getIMUYaw());
        telemetryGlobal.update();
    }

    public void setPID(double kP, double kI, double kD){
        turnPID = new UniversalPID(kP, kI, kD);
        turnPID.setFinishedConditions(0.5, 2); //Adjust after tuning PID
    }

    private double getIMUYaw(){
        /*
        Pair<Integer, Integer> curPos = mecanumdrivetrain.getPos();
        double ticksOnCircumference = (double) ((curPos.first - initialPos.first) + (initialPos.second - curPos.second)) /2;
        double distOnCircumference = ticksToDistance(ticksOnCircumference);
        double angle = (360*distOnCircumference)/(Math.PI*wheelBase);
        angle = angle%360;
        angle = (angle<0) ? angle+360 : angle;
        return angle;
         */
        return -pinpointComputer.getHeading(AngleUnit.DEGREES);
    }
}
