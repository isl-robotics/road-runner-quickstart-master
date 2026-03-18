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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Init;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.utilities.GlobalVars;

@Config
@Autonomous
public final class SplineTest extends LinearOpMode {

    public double mmToIn (double mm){
        return mm*0.03937;
    }
    /*
    public class Intake {
        private DcMotor intake;
        private Servo gate;
        private DcMotorEx launcher;

        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(DcMotor.class, "IntakeMotor");
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setDirection(DcMotorSimple.Direction.REVERSE);


            gate = hardwareMap.get(Servo.class, "Gate");
            launcher = hardwareMap.get(DcMotorEx.class, "LauncherMotor");
        }

        public class Shoot implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gate.setPosition(0.4);
                intake.setPower(-1);
                return false;
            }
        }
        public Action shoot(){
            return new Shoot();
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

     */

    @Override
    public void runOpMode() throws InterruptedException {

        //Intake intake = new Intake(hardwareMap);
        Pose2d beginPose = new Pose2d(61,  -14.5, Math.PI);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        TrajectoryActionBuilder traj1 = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(50, -20), -Math.toRadians(160))
                .waitSeconds(1);

        TrajectoryActionBuilder traj2 = traj1.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(mmToIn(900), -35,-Math.toRadians(90)), -Math.toRadians(90))
                //.waitSeconds(1)
                ;

        TrajectoryActionBuilder traj3 = traj2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(mmToIn(900),-65),new TranslationalVelConstraint(25))
                .waitSeconds(0.5);

        TrajectoryActionBuilder traj4 = traj3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(50, -20), -Math.toRadians(160))
                .waitSeconds(1);

        TrajectoryActionBuilder traj5 = traj4.endTrajectory().fresh()
                .splineToLinearHeading(new Pose2d(mmToIn(350), -35,-Math.toRadians(90)), -Math.toRadians(90))
                //.waitSeconds(1)
                ;

        TrajectoryActionBuilder traj6 = traj5.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(mmToIn(400),-65),new TranslationalVelConstraint(25))
                .waitSeconds(0.5);

        TrajectoryActionBuilder traj7 = traj6.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(50, -20), -Math.toRadians(160))
                .waitSeconds(1);

        TrajectoryActionBuilder traj8 = traj7.endTrajectory().fresh()
                .strafeTo(new Vector2d(30, -20))
                .waitSeconds(5);
        TrajectoryActionBuilder traj9 = traj8.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(61, -14.5), Math.PI)
                .waitSeconds(1);

        waitForStart();


    for (int i = 1; i<5; i++) {
        Actions.runBlocking(
                new SequentialAction(
                        traj1.build(),
                        //shoot
                        traj2.build(),
                        //intake.intakeIn(),
                        traj3.build(),
                        //intake.intakeOut(),
                        traj4.build(),
                        //shoot
                        traj5.build(),
                        //intake.intakeIn(),
                        traj6.build(),
                        //intake.intakeOut(),
                        traj7.build(),
                        //shoot
                        traj8.build(),
                        traj9.build()

                )
        );
    }

            /*
            Actions.runBlocking(
                drive.actionBuilder(beginPose)

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
