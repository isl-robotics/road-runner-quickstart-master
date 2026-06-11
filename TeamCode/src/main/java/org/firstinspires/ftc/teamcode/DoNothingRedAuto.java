package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utilities.Team;


@Autonomous(preselectTeleOp = "RedDrive")
public class DoNothingRedAuto extends AutoInit{
    @Override
    public void setTeam(){
        Team.set(Team.RED);
    }

    @Override
    public void extraInit(){
        Pose2d beginPose = new Pose2d(61,  -14.5, Math.PI);
        drive = new MecanumDrive(hardwareMap, beginPose);
    }
    @Override
    protected void runStrategy() {
        while(opModeIsActive()){
            pause(0.01);
        }
    }
}
