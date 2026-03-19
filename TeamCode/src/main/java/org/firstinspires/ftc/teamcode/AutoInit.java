package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public abstract class AutoInit extends Init{
    public double mmToIn (double mm){
        return mm*0.03937;
    }

    protected Action openGate;
    protected Action closeGate;
    protected Action intake;
    protected Action stopIntake;
    protected Action intakeGate;
    protected Action stopGate;
    protected Action startLauncher;
    protected Action stopLauncher;

    protected class OpenGate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            gateServo.setPosition(0.07);
            return false;
        }
    }

    protected class CloseGate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            gateServo.setPosition(0.2);
            return false;
        }
    }

    protected class Intake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeMotor.setPower(1);
            return false;
        }
    }

    protected class StopIntake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeMotor.setPower(0);
            return false;
        }
    }

    protected class IntakeGate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            gateMotor.setPower(-1);
            return false;
        }
    }

    protected class StopGate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            gateMotor.setPower(0);
            return false;
        }
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

    protected class StopLauncher implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            launcherController.setVelocity(0);
            return false;
        }
    }

    @Override
    protected void initActions(){
        openGate = new OpenGate();
        closeGate = new CloseGate();
        intake = new Intake();
        stopIntake = new StopIntake();
        intakeGate = new IntakeGate();
        stopGate = new StopGate();
        startLauncher = new StartLauncher();
        stopLauncher = new StopLauncher();
    }
}
