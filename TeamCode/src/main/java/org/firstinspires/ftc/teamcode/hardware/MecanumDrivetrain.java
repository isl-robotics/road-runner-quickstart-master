package org.firstinspires.ftc.teamcode.hardware;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class MecanumDrivetrain {
    private final DcMotor FL;
    private final DcMotor FR;
    private final DcMotor BL;
    private final DcMotor BR;

    private final double minPower = 0.07;

    private double axial;
    private double lateral;
    private double yaw;

    public MecanumDrivetrain(DcMotor FL, DcMotor FR, DcMotor BL, DcMotor BR){

        this.FL = FL;
        this.FR = FR;
        this.BL = BL;
        this.BR = BR;

        try {
            FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }catch (NullPointerException e){
            throw new RuntimeException("MOTORS ARE NULL");
        }

        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

        this.FL.setDirection(DcMotor.Direction.REVERSE);
        this.BL.setDirection(DcMotor.Direction.REVERSE);
        this.FR.setDirection(DcMotor.Direction.REVERSE);
        this.BR.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior zeroPowerBehaviour){
        FL.setZeroPowerBehavior(zeroPowerBehaviour);
        FR.setZeroPowerBehavior(zeroPowerBehaviour);
        BL.setZeroPowerBehavior(zeroPowerBehaviour);
        BR.setZeroPowerBehavior(zeroPowerBehaviour);
    }

    public void setPowers(double flPower, double frPower, double blPower, double brPower){
        double max = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
                Math.max(Math.abs(blPower), Math.abs(brPower)));

        if (max > 1.0) {
            flPower /= max;
            frPower /= max;
            blPower /= max;
            brPower /= max;
        }

        flPower = Range.clip(flPower, -1, 1);
        frPower = Range.clip(frPower, -1, 1);
        blPower = Range.clip(blPower, -1, 1);
        brPower = Range.clip(brPower, -1, 1);

        FL.setPower(flPower);
        FR.setPower(frPower);
        BL.setPower(blPower);
        BR.setPower(brPower);
    }

    public void setPowers(double lPower, double rPower){ // For backwards compatibility
        lPower = Range.clip(lPower, -1, 1);
        rPower = Range.clip(rPower, -1, 1);
        setPowers(lPower, rPower, lPower, rPower);
    }

    public void setDiagonals(double d1, double d2){
        setPowers(d1, d2, d2, d1);
    }
    public void setDiagonals(double d1, double d2, double rot){
        rot = Range.clip(rot, -1, 1);
        setPowers(d1+rot, d2-rot, d2+rot, d1-rot);
    }

    public void setOrtho(double x, double y){
        setDiagonals(y+x, y-x);
    }
    public void setOrthoAbs(double x, double y, double robotHeading){
        robotHeading = Math.toRadians(robotHeading);
        double x2 = Math.cos(-robotHeading)*x - Math.sin(-robotHeading)*y;
        double y2 = Math.sin(-robotHeading)*x + Math.cos(-robotHeading)*y;
        setDiagonals(y2+x2, y2-x2);
    }

    public void setOrthoAbs(double x, double y, double rot, double robotHeading){
        robotHeading = Math.toRadians(robotHeading);
        double x2 = Math.cos(-robotHeading)*x - Math.sin(-robotHeading)*y;
        double y2 = Math.sin(-robotHeading)*x + Math.cos(-robotHeading)*y;
        setDiagonals(y2+x2, y2-x2, rot);
    }

    public void setOrtho(double x, double y, double rot){
        setDiagonals(y+x, y-x, rot);
    }

    public void setPolar(double r, double th){
        setOrtho(r*Math.cos(Math.toRadians(th)), r*Math.sin(Math.toRadians(th)));
    }
    public void setPolar(double r, double th, double rot){
        setOrtho(r*Math.cos(Math.toRadians(th)), r*Math.sin(Math.toRadians(th)), rot);
    }

    public void rotate(double power){
        power = Range.clip(power, -1, 1);
        setPowers(power,-power);
    }

    public Pair<Integer, Integer> getPos(){ // For backwards compatibility
        int L = Math.round((float) (FL.getCurrentPosition() + BL.getCurrentPosition()) /2);
        int R = Math.round((float) (FR.getCurrentPosition() + BR.getCurrentPosition()) /2);
        return new Pair<Integer, Integer>(L, R);
    }

    public int[] getPosMecanum(){
        return new int[]{FL.getCurrentPosition(), FR.getCurrentPosition(), BL.getCurrentPosition(), BR.getCurrentPosition()};
    }
}
