package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(preselectTeleOp = "Drive")
public class DoNothingBlueAuto extends AutoInit{
    @Override
    public void extraInit(){
        Pose2d beginPose = new Pose2d(61,  -14.5, Math.PI);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
    }
    @Override
    protected void runStrategy() {
        while(opModeIsActive()){
            pause(0.01);
        }
    }
}
