package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.utilities.GlobalVars;

public abstract class AutoInit extends Init{

    protected MecanumDrive drive;

    @Override
    protected void extraDeInit(){
        GlobalVars.robotPos = drive.getPose();
    }

    public double mmToIn (double mm){
        return mm*0.03937;
    }

    protected class LowerKicker implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            kickerServo.setPosition(0.41);
            return false;
        }
    }

    protected Action lowerKickerAction(){
        return new LowerKicker();
    }

    protected class RaiseKicker implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            kickerServo.setPosition(0.6);
            return false;
        }
    }


    protected Action raiseKickerAction(){
        return new RaiseKicker();
    }

    protected class MediumKicker implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            kickerServo.setPosition(0.5);
            return false;
        }
    }

    protected Action mediumKickerAction(){
        return new MediumKicker();
    }

    protected class Intake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeMotor.setPower(1);
            gateMotor.setPower(-1);
            return false;
        }
    }

    protected Action intake(){
        return new Intake();
    }

    protected class StopIntake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeMotor.setPower(0);
            gateMotor.setPower(0);
            return false;
        }
    }

    protected Action stopIntake(){
        return new StopIntake();
    }

    protected class IntakeGate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            gateMotor.setPower(-1);
            return false;
        }
    }

    protected Action intakeGate(){
        return new IntakeGate();
    }

    protected class StopGate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            gateMotor.setPower(0);
            return false;
        }
    }

    protected Action stopGate(){
        return new StopGate();
    }

    protected class PrepLauncher implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            launcherController.setVelocity(launchingControlPoints.get(321));
            return false;
        }
    }

    protected Action prepLauncher(){
        return new PrepLauncher();
    }

    protected class IntakeLauncher implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            launcherController.setVelocity(200);
            return false;
        }
    }

    protected Action intakeLauncher(){
        return new IntakeLauncher();
    }

    protected class StartLauncher implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            Pair<Double, Double> distAndBearing = aprilTagDetector.getGoalDistAndBearing();
            if(distAndBearing != null){
                Double tagDist = distAndBearing.first;
                double speed = launchingControlPoints.get(tagDist);
                speed = Math.round(speed/20)*20;  // Round to nearest 20
                launcherController.setVelocity(speed);
                double start = clock.seconds();
                double now = clock.seconds();
                while((Math.abs(launcherController.getVelocity()-speed)>30) && opModeIsActive() && (now-start < 3)){
                    pause(.02);
                    now= clock.seconds();
                }
            }
            return false;
        }
    }

    protected Action startLauncher(){
        return new StartLauncher();
    }

    protected class Launch implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            boolean launchSequence = true;
            double start = clock.seconds();
            double now;
            while(launchSequence && opModeIsActive()){
                now = clock.seconds()-start;
                if (now>=1.7){
                    gateMotor.setPower(0);
                    intakeMotor.setPower(0);

                    launchSequence = false;
                } else if (now>=1.2) {
                    gateMotor.setPower(-1);
                    intakeMotor.setPower(1);
                } else if (now >= 0.8) {
                    mediumKicker();
                    gateMotor.setPower(-1);
                }else{
                   // raiseKicker();
                    kickerServo.setPosition(0.6);
                }
                pause(0.01);
            }
            return false;
        }
    }

    protected Action launch(){
        return new Launch();
    }

    protected class StopLauncher implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            launcherController.setVelocity(100);
            return false;
        }
    }

    protected Action stopLauncher(){
        return new StopLauncher();
    }
}
