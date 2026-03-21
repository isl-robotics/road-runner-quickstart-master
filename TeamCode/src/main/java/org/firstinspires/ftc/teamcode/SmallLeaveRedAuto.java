package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utilities.Team;


@Autonomous(preselectTeleOp = "RedDrive")
public class SmallLeaveRedAuto extends AutoInit{
    private TrajectoryActionBuilder traj1;

    @Override
    public void setTeam(){
        Team.set(Team.RED);
    }

    @Override
    public void extraInit(){
        Pose2d beginPose = new Pose2d(61,  -14.5, Math.PI);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        traj1 = drive.actionBuilder(beginPose)
                .strafeToConstantHeading(new Vector2d(58, -14.5-26))
                .waitSeconds(1);

    }
    @Override
    protected void runStrategy() {
        Actions.runBlocking(
                new SequentialAction(
                        traj1.build(),
                        closeGate()
                )
        );
    }
}
