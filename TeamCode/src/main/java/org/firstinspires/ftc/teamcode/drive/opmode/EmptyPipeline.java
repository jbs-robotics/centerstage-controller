package org.firstinspires.ftc.teamcode.drive.opmode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class EmptyPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    static HardwareMap hardwareMap;
    Mat mat = new Mat();
    private static int NUM_CLASSES=3;
    ;
    private Bitmap bitmap;
    private int rgbIndex = 2;
    private float[][] output = new float[1][NUM_CLASSES];
    private int width = 40, height = 40;
    public EmptyPipeline(Telemetry telemetry, HardwareMap hardwareMap, int rgbI) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.rgbIndex = rgbI;
    }
    @Override
    public void init(Mat frame) {
        telemetry.addData("Pipeline: ", "initialized");
        telemetry.update();
    }
    private char prediction = 'c';
    @Override
    public Mat processFrame(Mat input) {
        mat = input;
        return input;
    }
}