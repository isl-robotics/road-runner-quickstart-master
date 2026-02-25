package org.firstinspires.ftc.teamcode.hardware.camera;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.features2d.SimpleBlobDetector;
import org.opencv.features2d.SimpleBlobDetector_Params;
import org.opencv.imgproc.Imgproc;

public class ArtifactProcessor implements VisionProcessor {
    private final SimpleBlobDetector_Params params = new SimpleBlobDetector_Params();

    private SimpleBlobDetector blobDetector;

    public Scalar purpleMin = new Scalar(130,76,153);
    public Scalar purpleMax = new Scalar(160,255,230);
    public Scalar greenMin = new Scalar(53,89,153);
    public Scalar greenMax = new Scalar(73,255,204);

    public Scalar imgDim;

    Mat hsv = new Mat();
    Mat purpleMask = new Mat();
    Mat purpleOnly = new Mat();
    Mat greenMask = new Mat();
    Mat greenOnly = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

        imgDim = new Scalar(width, height);

        params.set_filterByArea(false);
        params.set_filterByInertia(false);
        params.set_filterByColor(false);

        params.set_filterByCircularity(true);
        params.set_minCircularity(0.6f);
        params.set_maxCircularity(1f);

        params.set_filterByConvexity(true);
        params.set_minConvexity(0.3f);
        params.set_maxConvexity(1f);

        blobDetector = SimpleBlobDetector.create(params);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsv, purpleMin, purpleMax, purpleMask);
        Core.bitwise_and(frame, frame, purpleOnly, purpleMask);

        Core.inRange(hsv, greenMin, greenMax, greenMask);
        Core.bitwise_and(frame, frame, greenOnly, greenMask);

        return purpleOnly;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }
}
