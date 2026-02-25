package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.util.ElapsedTime;

public class UniversalPID {
    private double kP, kI, kD;
    public double minVal = 0;
    public double maxVal = 1;
    private double integralSum = 0;
    private static final ElapsedTime clock = GlobalVars.clock;
    private double timeAtLastCalc = 0;
    private double lastError = 0;
    private double inTime = 0.5;
    private double withinError = 5;
    private double timeSpentAtTarget = 0;
    private double filteredD = 0;

    public UniversalPID(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public UniversalPID(double kP, double kI, double kD, double inTime, double withinError){
        this(kP, kI, kD);
        this.inTime = inTime;
        this.withinError = withinError;
    }

    public void setFinishedConditions(double inTime, double withinError){
        this.inTime = inTime;
        this.withinError = withinError;
    }

    public void setMinMaxVal(double minVal, double maxVal){
        this.minVal = minVal;
        this.maxVal = maxVal;
    }

    public  void setPID(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void clearIntegralSum(){integralSum = 0;}
    public void setIntegralSum(double integralSum){this.integralSum = integralSum;}

    public void reset(double error) {
        this.lastError = error;
        integralSum = 0;
        timeAtLastCalc = clock.seconds();
        filteredD = 0;
        timeSpentAtTarget = 0;
    }

    public double compute(double error){

        double output = 0;

        double timeNow = clock.seconds();
        double dE = error-lastError;
        double dT = timeNow-timeAtLastCalc;

        output = error*kP;

        integralSum += dT*error;
        output += integralSum*kI;

        double rawD = kD*(dE/dT);
        output += rawD;

        if(Math.copySign(1, error) != Math.copySign(1, lastError)){
            clearIntegralSum();
        }

        timeAtLastCalc = timeNow;
        lastError = error;

        if(Math.abs(error) <= withinError){
            timeSpentAtTarget += dT;
        }else{
            timeSpentAtTarget = 0;
        }

        if(Math.abs(output) > maxVal){
            output = Math.copySign(maxVal, output);
        }if(Math.abs(output) < minVal){
            output = Math.copySign(minVal, output);
        }

        return output;
    }

    public boolean isDone(){
        return timeSpentAtTarget >= inTime;
    }
}