package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(preselectTeleOp = "Drive")
public class BlueAuto2Gate extends AutoInit{
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
        Pose2d beginPose = new Pose2d(-56,  -45, -Math.toRadians(135));
        drive = new MecanumDrive(hardwareMap, beginPose);

        traj1 = drive.actionBuilder(beginPose)
            //    .strafeToConstantHeading(new Vector2d(mmToIn(-300), -20))
                .strafeToLinearHeading(new Vector2d(mmToIn(-250), -15), -Math.toRadians(140));
        ;
        traj2 = traj1.endTrajectory().fresh()
      //          .waitSeconds(0.3)
          //      .strafeToLinearHeading(new Vector2d(mmToIn(-300), -55), -Math.toRadians(90), new TranslationalVelConstraint(25));
             .strafeToLinearHeading(new Vector2d(mmToIn(-300), -30), -Math.toRadians(90));
        //  .waitSeconds(0.3)

        traj3 = traj2.endTrajectory().fresh()
          //      .strafeToConstantHeading(new Vector2d(mmToIn(-200),-55),new TranslationalVelConstraint(25))
                .strafeToConstantHeading(new Vector2d(mmToIn(-300),-55))
             //   .waitSeconds(0.2)
                ;
        traj4 = traj3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(mmToIn(-250), -15),-Math.toRadians(140))
          //      .strafeToLinearHeading(new Vector2d(-7, -16),-Math.toRadians(142))
                //        .waitSeconds(0.5)
              ;

        traj5 = traj4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(mmToIn(170), -30), -Math.toRadians(90))
                //.waitSeconds(1)
                ;

        traj6 = traj5.endTrajectory().fresh()
            //    .strafeToConstantHeading(new Vector2d(mmToIn(-200),-55),new TranslationalVelConstraint(25))
                .strafeToConstantHeading(new Vector2d(mmToIn(170),-63))
         //       .strafeToConstantHeading(new Vector2d(mmToIn(320),-20))
          //      .waitSeconds(0.2)
                ;
        traj7 = traj6.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(mmToIn(-250), -15), -Math.toRadians(140))
           //     .strafeToLinearHeading(new Vector2d(-7, -16), -Math.toRadians(142))
         //       .waitSeconds(0.5)
            ;

        traj8 = traj7.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(mmToIn(920), -25), -Math.toRadians(90))
        ;
        traj9= traj8.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(mmToIn(920), -60), -Math.toRadians(90))
           //     .splineToLinearHeading(new Pose2d(mmToIn(860), -50,-Math.toRadians(90)), -Math.toRadians(90), new TranslationalVelConstraint(25))
            //    .waitSeconds(1)
                ;
        traj10 = traj9.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(mmToIn(-250), -15), -Math.toRadians(140))
               // .strafeToLinearHeading(new Vector2d(-7, -15),-Math.toRadians(142))
        //      .waitSeconds(0.3)
                  ;
        traj11 = traj10.endTrajectory().fresh()
           //     .strafeToLinearHeading(new Vector2d(-7, -18), -Math.toRadians(130));
                .strafeTo(new Vector2d(-7,-18));
        //   .strafeToLinearHeading(new Vector2d(50, -15), -Math.toRadians(160));

      //  traj11 = traj10.endTrajectory().fresh()

    }
    @Override
    protected void runStrategy() {
        Actions.runBlocking(
                new SequentialAction(
                        //PRE-LOADED BALLS
                        prepLauncherClose(),
                        traj1.build(),
                     //   startLauncher(),
                        launch(),
                        stopLauncher(),
                        //1ST ROW
                        traj2.build(),
                        intake(),
                        intakeLauncher(),
                        lowerKickerAction(),
                        traj3.build(),
                        stopIntake(),
                        prepLauncherClose(),
                        traj4.build(),
                    //    startLauncher(),
                        launch(),
                        stopLauncher(),
                        //2ND ROW
                        traj5.build(),
                        intakeLauncher(),
                        intake(),
                        traj6.build(),
                        stopIntake(),
                        prepLauncherClose(),
                        traj7.build(),
                    //    startLauncher(),
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
                    //    startLauncher(),
                        launch(),
                        //LEAVE TRIANGLE
                        traj11.build(),
                        stopLauncher(),
                        stopIntake()

                )
        );
    }
}
