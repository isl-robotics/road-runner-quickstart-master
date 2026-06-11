package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(preselectTeleOp = "Drive")
public class BlueAuto extends AutoInit{
    private TrajectoryActionBuilder traj1;
    private TrajectoryActionBuilder traj2;
    private TrajectoryActionBuilder traj3;
    private TrajectoryActionBuilder traj4;
    private TrajectoryActionBuilder traj5;
    private TrajectoryActionBuilder traj6;
    private TrajectoryActionBuilder traj7;
    private TrajectoryActionBuilder traj8;
    private TrajectoryActionBuilder traj9;
    private TrajectoryActionBuilder traj10;
    private TrajectoryActionBuilder traj11;

    @Override
    public void extraInit(){
        Pose2d beginPose = new Pose2d(61,  -14.5, Math.PI);
        drive = new MecanumDrive(hardwareMap, beginPose);

        traj1 = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(50, -15), -Math.toRadians(155))
                .waitSeconds(0.5);

        traj2 = traj1.endTrajectory().fresh()
             //   .splineToLinearHeading(new Pose2d(mmToIn(900), -38,-Math.toRadians(90)), -Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(mmToIn(920), -30,-Math.toRadians(90)), -Math.toRadians(90))
                //.splineToLinearHeading(new Pose2d(mmToIn(920), -50,-Math.toRadians(90)), -Math.toRadians(90), new TranslationalVelConstraint(25))
        //  .waitSeconds(0.3)
                ;

        traj3 = traj2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(mmToIn(920),-55),new TranslationalVelConstraint(25))
         //       .waitSeconds(0.2)
                ;

        traj4 = traj3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(50, -15), -Math.toRadians(158))
                .waitSeconds(0.5)
                ;

        traj5 = traj4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(mmToIn(280), -30), -Math.toRadians(90))
              //  .strafeToLinearHeading(new Vector2d(40, -20), -Math.toRadians(160))
            //    .splineToLinearHeading(new Pose2d(mmToIn(300), -38,-Math.toRadians(90)), -Math.toRadians(90))
                //.waitSeconds(1)
                ;

        traj6 = traj5.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(mmToIn(280),-55),new TranslationalVelConstraint(25))
            //    .waitSeconds(0.2)
            ;

        traj7 = traj6.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(50, -15), -Math.toRadians(158))
                .waitSeconds(0.5)
                ;

        traj8 = traj7.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(mmToIn(-330), -28), -Math.toRadians(90));
        //   .strafeTo(new Vector2d(30, -20)) //CODE FOR LEAVING ZONE
             //   .splineToLinearHeading(new Pose2d(mmToIn(-300), -38, -Math.toRadians(90)), -Math.toRadians(90));

        traj9 = traj8.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(mmToIn(-330),-55),new TranslationalVelConstraint(25));

        traj10 = traj9.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(mmToIn(-300), -18), -Math.toRadians(140))
                .waitSeconds(0.1);

        traj11 = traj10.endTrajectory().fresh()
                .strafeTo(new Vector2d(mmToIn(-200),-38));



    }
    @Override
    protected void runStrategy() {
        Actions.runBlocking(
                new SequentialAction(
                        //PRE-LOADED BALLS
                        prepLauncher(),
                        traj1.build(),
                        //startLauncher(),
                        launch(),
                        stopLauncher(),
                        //1ST ROW
                        traj2.build(),
                        intakeLauncher(),
                        lowerKickerAction(),
                        intake(),
                        traj3.build(),
                        stopIntake(),
                        prepLauncher(),
                        traj4.build(),
                        launch(),
                        stopLauncher(),
                        //2ND ROW
                        traj5.build(),
                        intakeLauncher(),
                        intake(),
                        traj6.build(),
                        stopIntake(),
                        prepLauncher(),
                        traj7.build(),
                       // startLauncher(),
                        launch(),
                        stopLauncher(),
                        //3RD ROW
                        traj8.build(),
                        intakeLauncher(),
                        lowerKickerAction(),
                        intake(),
                        traj9.build(),
                        stopIntake(),
                        prepLauncherClose(),
                        traj10.build(),
                    //startLauncher(),
                        launch(),
                        //LEAVE TRIANGLE
                        traj11.build(),
                        stopLauncher(),
                        stopIntake()
                )
        );
    }
}
