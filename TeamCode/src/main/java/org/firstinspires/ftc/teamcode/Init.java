package org.firstinspires.ftc.teamcode;

import android.util.Pair;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

    protected Servo gateServo;

    protected DcMotorEx launcherMotor1;
    protected DcMotorEx launcherMotor2;
    protected InterpLUT launchingControlPoints;

    protected UniversalPID goalAlignmentPID = new UniversalPID(-0.08,0,-0.009, 0.5, 2);
    protected LauncherController launcherController;
    protected MecanumDrivetrain mecanumDrivetrain;
    protected MecanumDrivetrainController mecanumDrivetrainController;

    protected GoBildaPinpointDriver pinpointComputer;
    protected double alignmentPower;

    public void extraInit(){}

    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
        gateServo = hardwareMap.get(Servo.class, "Gate");

        launcherController = new LauncherController(launcherMotor1, launcherMotor2, gateMotor);

        intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mecanumDrivetrainController = new MecanumDrivetrainController(mecanumDrivetrain, pinpointComputer);
        clock.reset();

        setTeam();
        initLauncherControlPoints();
        initActions();
        initCamera();
        initPinpointComputer();

        extraInit();

        waitForStart();
        runStrategy();

        mecanumDrivetrain.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
        mecanumDrivetrain.setDiagonals(0,0);
        stopVisionPortal();
    }

    protected abstract void runStrategy();

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

        launchingControlPoints.add(5, 1520);
        launchingControlPoints.add(10, 1520);
        launchingControlPoints.add(20, 1520);
        launchingControlPoints.add(30, 1520);
        launchingControlPoints.add(40, 1520);
        launchingControlPoints.add(50, 1520);
        launchingControlPoints.add(60, 1520);
        launchingControlPoints.add(70, 1520);
        launchingControlPoints.add(80, 1520);
        launchingControlPoints.add(90, 1520);
        launchingControlPoints.add(100, 1520);
        launchingControlPoints.add(110, 1520);
        launchingControlPoints.add(120, 1520);
        launchingControlPoints.add(130, 1520);
        launchingControlPoints.add(140, 1520);
        launchingControlPoints.add(150, 1520);
        launchingControlPoints.add(160, 1520);
        launchingControlPoints.add(170, 1500);
        launchingControlPoints.add(180, 1520);
        launchingControlPoints.add(190, 1520);
        launchingControlPoints.add(200, 1520);
        launchingControlPoints.add(210, 1540);
        launchingControlPoints.add(220, 1540);
        launchingControlPoints.add(230, 1540);
        launchingControlPoints.add(240, 1540);
        launchingControlPoints.add(250, 1560);
        launchingControlPoints.add(260, 1560);
        launchingControlPoints.add(270, 1560);
        launchingControlPoints.add(280, 1560);
        launchingControlPoints.add(290, 1580);
        launchingControlPoints.add(300, 1600);
        launchingControlPoints.add(310, 1600);
        launchingControlPoints.add(320, 1620);
        launchingControlPoints.add(330, 1620);
        launchingControlPoints.add(340, 1640);
        launchingControlPoints.add(350, 1640);
        launchingControlPoints.add(360, 1640);
        launchingControlPoints.add(370, 1640);
        launchingControlPoints.add(380, 1640);

        launchingControlPoints.createLUT();
    }

    private void initPinpointComputer(){
        pinpointComputer = hardwareMap.get(GoBildaPinpointDriver.class, "PinpointComputer");
        pinpointComputer.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpointComputer.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
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

    public Pair<Double, Boolean> alignToGoal(){
        Pair<Double, Double> distAndBearing = aprilTagDetector.getGoalDistAndBearing();
        if (distAndBearing != null) {
            double error = -distAndBearing.second-4d;
            if(Math.abs(error)<1){
                alignmentPower = 0;
            }
            alignmentPower = Range.clip(goalAlignmentPID.compute(error), -0.4, 0.4);
            return new Pair<>(distAndBearing.second, goalAlignmentPID.isDone());
        } else {
            alignmentPower = 0;
            return null;
        }
    }

    public void launchAtDist(double tagDist){
        //double speed = (6*0.00001)*Math.pow(tagDist, 3)-0.0501*Math.pow(tagDist, 2)+13.511*tagDist+405.1;
        double speed = launchingControlPoints.get(tagDist);
        speed = Math.round(speed/20)*20;  // Round to nearest 20
        launcherController.setVelocity(speed);
        /*
        double start = clock.seconds();
        double now = clock.seconds();
        while((Math.abs(launcherController.launcherMotor.getVelocity()-speed)>30) && opModeIsActive() && (now-start < 3)){
            pause(.02);
            now= clock.seconds();
        }
        gate.setPosition(0.4);
        pause(0.2);
        intakeMotor.setPower(-1);
        pause(0.5);
         */
    }
}
