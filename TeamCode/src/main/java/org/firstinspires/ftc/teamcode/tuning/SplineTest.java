package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(61,  -14.5, Math.PI);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(beginPose)

                        //.splineTo(new Vector2d(30, 30), Math.PI / 2)
                        //.splineTo(new Vector2d(0, 60), Math.PI)
                        //.splineToLinearHeading(new Pose2d(0, 0, 0), Math.PI)
                        //.waitSeconds(10)
                        /*
                        .strafeToLinearHeading(new Vector2d(50, -14.5), -Math.toRadians(160))
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(mmToIn(900), -14.5), -Math.toRadians(90))
                        //.waitSeconds(0)
                        .lineToY(-50)
                        .waitSeconds(0)
                        .strafeToLinearHeading(new Vector2d(50, -14.5), -Math.toRadians(160))
                        .waitSeconds(1)
                        .strafeToLinearHeading(new Vector2d(mmToIn(300), -14.5), -Math.toRadians(90))
                        .waitSeconds(0)
                        .lineToY(-50)
                        .waitSeconds(0)
                        .strafeToLinearHeading(new Vector2d(50, -14.5), -Math.toRadians(160))
                        .waitSeconds(1)
                        .splineToLinearHeading(new Pose2d(61,-14.5,Math.PI), 0)

                         */
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
                        .strafeToLinearHeading(new Vector2d(40,-14.5), Math.PI)
                        .waitSeconds(5)
                        .strafeToLinearHeading(new Vector2d(61,-14.5), Math.PI)
                        .build());
        }
        else {
            throw new RuntimeException();
        }
    }

    public double mmToIn (double mm){
        return mm*0.03937;
    }
}
