package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Mat;


@Autonomous(preselectTeleOp = "Drive")
public class BigLeaveBlueAuto extends AutoInit{
    private TrajectoryActionBuilder traj1;

    @Override
    public void extraInit(){
        Pose2d beginPose = new Pose2d(-48,  -50.5, (5d/18d)*Math.PI);
        drive = new MecanumDrive(hardwareMap, beginPose);

        traj1 = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(-32.25, -39.52), Math.PI/2)
                .strafeToConstantHeading(new Vector2d(-32.25, -55.75))
                .waitSeconds(1);

    }
    @Override
    protected void runStrategy() {
        Actions.runBlocking(
                new SequentialAction(
                        traj1.build()
                )
        );
    }
}
