package com.example.MeepMeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) throws IOException {
        MeepMeep meepMeep = new MeepMeep(780);
        Pose2d beginPose = new Pose2d(-61,  -14.5, Math.PI);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(beginPose)
                        .strafeTo(new Vector2d(mmToIn(-300), -20))
                        .waitSeconds(0.5)
                        .strafeTo(new Vector2d(mmToIn(-300), -30))
                        .strafeTo(new Vector2d(mmToIn(-300),-56))
                        .strafeTo(new Vector2d(mmToIn(-300), -20))
                        .waitSeconds(0.5)
                        .strafeTo(new Vector2d(mmToIn(300), -30))
                        .strafeTo(new Vector2d(mmToIn(300),-55))
                        .strafeTo(new Vector2d(mmToIn(-300), -20))
                        .waitSeconds(0.5)
                        .strafeTo(new Vector2d(mmToIn(860), -20))
                        .strafeTo(new Vector2d(mmToIn(860), -50))
                        .strafeTo(new Vector2d(mmToIn(-300), -20))
                        .waitSeconds(0.3)
                        .strafeTo(new Vector2d(mmToIn(-200),-38))
                        .build());

        BufferedImage decodeField = ImageIO.read(new File("C:\\Users\\DELL\\Desktop\\FTC-202526\\Robot_Controller_Builds\\road-runner-quickstart-master\\MeepMeepTesting\\src\\main\\java\\com\\example\\MeepMeep\\decode-field.jpg"));
        meepMeep.setBackground(decodeField)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public static double mmToIn(double mm){
        return mm*0.03937;
    }
}