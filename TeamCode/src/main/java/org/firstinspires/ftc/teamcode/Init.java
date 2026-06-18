package org.firstinspires.ftc.teamcode;

import android.util.Pair;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.LauncherController;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrivetrainController;
import org.firstinspires.ftc.teamcode.hardware.camera.AprilTagDetector;
import org.firstinspires.ftc.teamcode.hardware.camera.ArtifactProcessor;
import org.firstinspires.ftc.teamcode.utilities.GlobalVars;
import org.firstinspires.ftc.teamcode.utilities.Team;
import org.firstinspires.ftc.teamcode.utilities.UniversalPID;
import org.firstinspires.ftc.vision.VisionPortal;

public abstract class Init extends LinearOpMode {
    protected VisionPortal visionPortal;
    protected ArtifactProcessor artifactProcessor = new ArtifactProcessor();
    protected AprilTagDetector aprilTagDetector = new AprilTagDetector();
    public static final ElapsedTime clock = GlobalVars.clock;
    protected DcMotor flMotor;
    protected DcMotor frMotor;
    protected DcMotor blMotor;
    protected DcMotor brMotor;
    protected DcMotor intakeMotor;
    protected DcMotor gateMotor;
    protected Servo kickerServo;


    protected DcMotorEx launcherMotor1;
    protected DcMotorEx launcherMotor2;
    protected InterpLUT launchingControlPoints;

    protected UniversalPID goalAlignmentPID = new UniversalPID(0.022, 0.011, 0.0003);
    protected LauncherController launcherController;
    protected MecanumDrivetrain mecanumDrivetrain;
    protected MecanumDrivetrainController mecanumDrivetrainController;

    protected GoBildaPinpointDriver pinpointComputer;
    protected PinpointLocalizer pinpointLocalizer;

    protected FtcDashboard dashboard;

    protected TelemetryPacket telemetryPacket;

    public void extraInit(){}

    public void runOpMode(){
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        GlobalVars.opModeIsActiveGlobal = this::opModeIsActive;
        GlobalVars.telemetryGlobal = telemetry;

        flMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        frMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        blMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        brMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");
        mecanumDrivetrain = new MecanumDrivetrain(flMotor, frMotor, blMotor, brMotor);

        launcherMotor1 = hardwareMap.get(DcMotorEx.class, "LauncherMotor1");
        launcherMotor2 = hardwareMap.get(DcMotorEx.class, "LauncherMotor2");

        gateMotor = hardwareMap.get(DcMotor.class, "GateMotor");
        kickerServo = hardwareMap.get(Servo.class, "Kicker");

        launcherController = new LauncherController(launcherMotor1, launcherMotor2, gateMotor);

        intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        initPinpointComputer();
        mecanumDrivetrainController = new MecanumDrivetrainController(mecanumDrivetrain, pinpointComputer);

        pinpointLocalizer = new PinpointLocalizer(hardwareMap, 0.0019558353279081, GlobalVars.robotPos);

        clock.reset();

        setTeam();
        initLauncherControlPoints();
        initActions();
        initCamera();

        extraInit();

        lowerKicker();

        waitForStart();
        runStrategy();

        extraDeInit();

        mecanumDrivetrain.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        mecanumDrivetrain.setDiagonals(0,0);
        stopVisionPortal();
    }

    protected abstract void runStrategy();

    protected void extraDeInit() {}

    private void initCamera(){
        VisionPortal.Builder myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        //myVisionPortalBuilder.addProcessor(artifactProcessor);
        myVisionPortalBuilder.addProcessor(aprilTagDetector.getAprilTagProcessor());
        myVisionPortalBuilder.setCameraResolution(new Size(640, 480));
        visionPortal = myVisionPortalBuilder.build();
    }

    public void stopVisionPortal(){
        if (visionPortal != null){
            visionPortal.close();
        }
    }

    private void initLauncherControlPoints(){
        launchingControlPoints = new InterpLUT();

        launchingControlPoints.add(0, 1560);
        launchingControlPoints.add(25, 1560);
        launchingControlPoints.add(50, 1560);
        launchingControlPoints.add(75, 1560);
        launchingControlPoints.add(100, 1560);
        launchingControlPoints.add(120, 1560);
        launchingControlPoints.add(150, 1580);
        launchingControlPoints.add(160, 1580);
        launchingControlPoints.add(170, 1580);
        launchingControlPoints.add(180, 1600);
        launchingControlPoints.add(190, 1580);
        launchingControlPoints.add(200, 1560);
        launchingControlPoints.add(210, 1560);
        launchingControlPoints.add(220, 1560);
        launchingControlPoints.add(230, 1560);
        launchingControlPoints.add(240, 1560);
        launchingControlPoints.add(250, 1560);
        launchingControlPoints.add(260, 1560);
        launchingControlPoints.add(270, 1580);
        launchingControlPoints.add(280, 1580);
        launchingControlPoints.add(290, 1620);
        launchingControlPoints.add(300, 1620);
        launchingControlPoints.add(310, 1640);
        launchingControlPoints.add(320, 1640);
        launchingControlPoints.add(330, 1680);
        launchingControlPoints.add(340, 1680);
        launchingControlPoints.add(350, 1720);
        launchingControlPoints.add(360, 1720);
        launchingControlPoints.add(370, 1760);
        launchingControlPoints.add(380, 1760);

        launchingControlPoints.createLUT();
    }

    private void initPinpointComputer(){
        pinpointComputer = hardwareMap.get(GoBildaPinpointDriver.class, "PinpointComputer");
        pinpointComputer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED); // TODO: find proper reversing for odometry
        pinpointComputer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpointComputer.setOffsets(-90.752,-92.734, DistanceUnit.MM);
        //pinpointComputer.setOffsets(-13.5,-37.7, DistanceUnit.CM);
        //pinpointComputer.resetPosAndIMU();
    }

    public void setTeam(){
        Team.set(Team.BLUE);
    }

    public void pause(double seconds){
        double start = clock.seconds();
        double now = clock.seconds();
        while( ((now-start)<seconds) && opModeIsActive() ){
            now = clock.seconds();
            try { Thread.sleep(5); } catch (InterruptedException e) {}
        }
    }

    protected void initActions(){
    }

    protected void lowerKicker(){kickerServo.setPosition(0.45);}
    protected void raiseKicker(){kickerServo.setPosition(0.6);}

    protected void mediumKicker(){kickerServo.setPosition(0.5);}

    public Pair<Double, Boolean> alignToGoal(){

        pinpointLocalizer.update();
        Pose2d robotPose = pinpointLocalizer.getPose();

        Vector2d goalPos = new Vector2d(-64, -60);
        if(Team.get() == Team.RED){
            goalPos = new Vector2d(-59.5, 62.5);
        }

        double targetHeading = Math.atan2(goalPos.y-robotPose.position.y, goalPos.x-robotPose.position.x);

        double errorAngle = Math.toDegrees(robotPose.heading.toDouble()-targetHeading);
        if(errorAngle > 180){
            errorAngle -= 360;
        }else if(errorAngle < -180){
            errorAngle += 360;
        }

        double alignmentPower = goalAlignmentPID.compute(errorAngle);

        /*
        telemetry.addData("Robot X (in)", robotPose.position.x);
        telemetry.addData("Robot Y (in)", robotPose.position.y);
        telemetry.addData("Encoder X (ticks)", pinpointComputer.getEncoderX());
        telemetry.addData("Encoder Y (ticks)", pinpointComputer.getEncoderY());
        telemetry.addData("currentHeading", Math.toDegrees(robotPose.heading.toDouble()));
        telemetry.addData("targetHeading", Math.toDegrees(targetHeading));
        telemetry.addData("errorAngle", errorAngle);
         */

        /*
        Canvas c = telemetryPacket.fieldOverlay();

        c.setStroke("#4CAF50");
        Drawing.drawRobot(c, new Pose2d(robotPose.position, targetHeading));
         */

        //c.setStroke("#3F51B5");
        //Drawing.drawRobot(c, robotPose);

        //dashboard.sendTelemetryPacket(telemetryPacket);

        return new Pair<>(alignmentPower, goalAlignmentPID.isDone());
    }

    public double getGoalDist(){
        pinpointLocalizer.update();
        Pose2d robotPose = pinpointLocalizer.getPose();

        Vector2d goalPos = new Vector2d(-60, -55);
        if(Team.get() == Team.RED){
            goalPos = new Vector2d(-59.5, 62.5);
        }
        return 2.54*Math.sqrt(Math.pow(goalPos.y-robotPose.position.y, 2)+Math.pow(goalPos.x-robotPose.position.x, 2));
    }

    public void launchAtDist(double tagDist){
        //double speed = (6*0.00001)*Math.pow(tagDist, 3)-0.0501*Math.pow(tagDist, 2)+13.511*tagDist+405.1;
        double speed = launchingControlPoints.get(tagDist);
        speed = Math.round(speed/20)*20;  // Round to nearest 20
        launcherController.setVelocity(speed);
/*
        double start = clock.seconds();
        double now = clock.seconds();
        while((Math.abs(launcherController.getVelocity()-speed)>30) && opModeIsActive() && (now-start < 3)){
            pause(.02);
            now= clock.seconds();
        }
        */
    }
}
