package org.firstinspires.ftc.teamcode.tuning;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous
public final class SplineTest extends LinearOpMode {

    public double mmToIn (double mm){
        return mm*0.03937;
    }
    public class Intake {
        private DcMotor intake;

        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotor.class, "IntakeMotor");
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        public class IntakeIn implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                    intake.setPower(1);
                    return false;
            }
        }
        public Action intakeIn(){
            return new IntakeIn();
        }
        public class IntakeOut implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(1);
                return false;
            }
        }
        public Action intakeOut(){
            return new IntakeOut();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(hardwareMap);
        Pose2d beginPose = new Pose2d(61,  -14.5, Math.PI);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        TrajectoryActionBuilder traj1 = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(50, -20), -Math.toRadians(160))
                .waitSeconds(1)
                .splineToLinearHeading(new Pose2d(mmToIn(900), -35,-Math.toRadians(90)), -Math.toRadians(90))
                .waitSeconds(1);

        TrajectoryActionBuilder traj2 = traj1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(mmToIn(900),-65),new TranslationalVelConstraint(25))
                .waitSeconds(1);

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        traj1.build(),
                        intake.intakeIn(),
                        traj2.build(),
                        intake.intakeOut()
                )
        );

            /*
            Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .strafeToLinearHeading(new Vector2d(50, -20), -Math.toRadians(160))
                        .waitSeconds(1)
                        .splineToLinearHeading(new Pose2d(mmToIn(300), -35,-Math.toRadians(90)), -Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(mmToIn(300),-65),new TranslationalVelConstraint(25))
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(50, -20), -Math.toRadians(160))
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(40,-20), Math.PI)
                        .turn(Math.toRadians(360*2))
                        .waitSeconds(5)
                        .strafeToLinearHeading(new Vector2d(61,-14.5), Math.PI)
                        .build());

             */

    }

}
