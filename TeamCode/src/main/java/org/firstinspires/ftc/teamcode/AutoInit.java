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

    protected class OpenGate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            kickerServo.setPosition(0.07);
            return false;
        }
    }

    protected Action openGate(){
        return new OpenGate();
    }

    protected class CloseGate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            kickerServo.setPosition(0.2);
            return false;
        }
    }


    protected Action closeGate(){
        return new CloseGate();
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
            gateMotor.setPower(-1);
            intakeMotor.setPower(1);
            pause(1.7);
            return false;
        }
    }

    protected Action launch(){
        return new Launch();
    }

    protected class StopLauncher implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            launcherController.setVelocity(0);
            return false;
        }
    }

    protected Action stopLauncher(){
        return new StopLauncher();
    }
}
