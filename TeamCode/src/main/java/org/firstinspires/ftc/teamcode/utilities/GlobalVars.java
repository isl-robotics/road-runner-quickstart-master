package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;

public class GlobalVars {
    public static final ElapsedTime clock = new ElapsedTime();
    public static void pauseGlobal(double seconds){
        double start = clock.seconds();
        double now = clock.seconds();
        while( ((now-start)<seconds) && opModeIsActiveGlobal.getAsBoolean() ){
            now = clock.seconds();
            try { Thread.sleep(5); } catch (InterruptedException e) {}
        }
    }

    public static BooleanSupplier opModeIsActiveGlobal;

    public static Telemetry telemetryGlobal;

    public static final int defaultLauncherSpeed = 0;
}
