package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous
public class BlueAuto2 extends AutoInit{
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
        Pose2d beginPose = new Pose2d(-61,  -14.5, Math.PI);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        traj1 = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(mmToIn(-300), -20), -Math.toRadians(140))
                .waitSeconds(0.5)
        ;
        traj2 = traj1.endTrajectory().fresh()
          //      .strafeToLinearHeading(new Vector2d(mmToIn(-300), -55), -Math.toRadians(90), new TranslationalVelConstraint(25));
             .strafeToLinearHeading(new Vector2d(mmToIn(-300), -30), -Math.toRadians(90));
        //  .waitSeconds(0.3)
                ;

        traj3 = traj2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(mmToIn(-300),-56),new TranslationalVelConstraint(25))
             //   .waitSeconds(0.2)
                ;
        traj4 = traj3.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(mmToIn(-300), -20), -Math.toRadians(140))
                .waitSeconds(0.5)
              ;

        traj5 = traj4.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(mmToIn(300), -30), -Math.toRadians(90))
                //.waitSeconds(1)
                ;

        traj6 = traj5.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(mmToIn(300),-55),new TranslationalVelConstraint(25))
          //      .waitSeconds(0.2)
                ;
        traj7 = traj6.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(mmToIn(-300), -20), -Math.toRadians(140))
                .waitSeconds(0.5);

        traj8 = traj7.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(mmToIn(860), -20), -Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(mmToIn(860), -50), -Math.toRadians(90))
           //     .splineToLinearHeading(new Pose2d(mmToIn(860), -50,-Math.toRadians(90)), -Math.toRadians(90), new TranslationalVelConstraint(25))
            //    .waitSeconds(1)
                ;
        traj9 = traj8.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(mmToIn(-300), -20), -Math.toRadians(140))
                .waitSeconds(0.3);
        traj10 = traj9.endTrajectory().fresh()
                .strafeTo(new Vector2d(mmToIn(-200),-38));
        //   .strafeToLinearHeading(new Vector2d(50, -15), -Math.toRadians(160));

      //  traj11 = traj10.endTrajectory().fresh()

    }
    @Override
    protected void runStrategy() {
        Actions.runBlocking(
                new SequentialAction(
                        prepLauncher(),
                        traj1.build(),
                        startLauncher(),
                        openGate(),
                        launch(),
                        closeGate(),
                        traj2.build(),
                    //    intake(),
                        traj3.build(),
                    //    stopIntake(),
                        traj4.build(),
                        startLauncher(),
                        openGate(),
                        launch(),
                    //    stopLauncher(),
                    //    stopIntake(),
                        closeGate(),
                        traj5.build(),
                        intake(),
                        traj6.build(),
                   //     stopIntake(),
                        traj7.build(),
                        startLauncher(),
                        openGate(),
                        launch(),
                        closeGate(),
                        traj8.build(),
                        traj9.build(),
                        startLauncher(),
                        openGate(),
                        launch(),
                        closeGate(),
                        traj10.build(),
                        stopLauncher(),
                        stopIntake()

                )
        );
    }
}
