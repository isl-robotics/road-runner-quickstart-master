package com.example.MeepMeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .splineToLinearHeading(new Pose2d(50, -14.5, -Math.toRadians(160)), -Math.toRadians(160))
                        .waitSeconds(1)
                        .splineToLinearHeading(new Pose2d(mmToIn(900), -35,-Math.toRadians(90)), -Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(mmToIn(900), -50,-Math.toRadians(90)), -Math.toRadians(90))
                        .waitSeconds(1)
                        .splineToLinearHeading(new Pose2d(50, -14.5, -Math.toRadians(160)), Math.toRadians(20))
                        .waitSeconds(1)
                        .splineToLinearHeading(new Pose2d(mmToIn(300), -35,-Math.toRadians(90)), -Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(mmToIn(300), -50,-Math.toRadians(90)), -Math.toRadians(90))
                        .waitSeconds(1)
                        .splineToLinearHeading(new Pose2d(50, -14.5, -Math.toRadians(160)), Math.toRadians(20))
                        .waitSeconds(1)
                        .build());


        meepMeep.setBackground(MeepMeep.Background.GRID_GRAY)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public static double mmToIn(double mm){
        return mm*0.03937;
    }
}