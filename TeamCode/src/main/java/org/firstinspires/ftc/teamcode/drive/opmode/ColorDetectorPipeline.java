package org.firstinspires.ftc.teamcode.drive.opmode;

import android.content.res.AssetFileDescriptor;
import android.graphics.Bitmap;
import org.opencv.core.Rect;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.FileInputStream;
import java.io.IOException;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;

public class ColorDetectorPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    static HardwareMap hardwareMap;
    Mat mat = new Mat();
    private String filepath = "blue.tflite";
    private static int NUM_CLASSES=3;
    ;
    private Bitmap bitmap;
    private int rgbIndex = 2;
    private float[][] output = new float[1][NUM_CLASSES];

    private static MappedByteBuffer loadModelFile(String modelPath) throws IOException {
        AssetFileDescriptor fileDescriptor = hardwareMap.appContext.getAssets().openFd(modelPath);
        FileInputStream inputStream = new FileInputStream(fileDescriptor.getFileDescriptor());
        FileChannel fileChannel = inputStream.getChannel();
        long startOffset = fileDescriptor.getStartOffset();
        long declaredLength = fileDescriptor.getDeclaredLength();
        return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength);
    }

    private int width = 40, height = 40;
    public ColorDetectorPipeline(Telemetry telemetry, HardwareMap hardwareMap, int rgbI) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.rgbIndex = rgbI;
    }
    public ColorDetectorPipeline(Telemetry telemetry, HardwareMap hardwareMap, int rgbI, int w, int h) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.rgbIndex = rgbI;
        this.width = w;
        this.height = h;
    }
    @Override
    public void init(Mat frame) {
        telemetry.addData("Pipeline: ", "initialized");
        telemetry.update();
    }
    private Point region1_pointA = new Point(
            0,
            120);
    private Point region1_pointB = new Point(
            40,
            160);
    private Point region2_pointA = new Point(
            (int)160,
            (int)120);
    private Point region2_pointB = new Point(
            200,
            160);

    final Scalar BLUE = new Scalar(0, 0, 255);
    private double[] region1Average = new double[3];
    private double[] region2Average = new double[3];
    private char prediction = 'c';
    private double threshold = 180, percentageThreshold = 0.47, percentRed1, percentRed2, percentBlue1, percentBlue2;
    @Override
    public Mat processFrame(Mat input) {
        mat = input;
//        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
//        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_HSV2RGB);
//        mat = input;
//        Imgproc.cvtColor(mat, input, Imgproc.COLOR_GRAY2RGB);
//        bitmap = Bitmap.createBitmap(input.cols(), input.rows(), Bitmap.Config.ARGB_8888);
//        tflite.run(bitmap, output);
//        prediction = getClassLabel(output);
        Rect REGION1_CROP = new Rect(
                (int) region1_pointA.x,
                (int) region1_pointA.y,
                (int) (region1_pointB.x - region1_pointA.x),
                (int) (region1_pointB.y - region1_pointA.y));
        Rect REGION2_CROP = new Rect(
                (int) region2_pointA.x,
                (int) region2_pointA.y,
                (int) (region2_pointB.x - region2_pointA.x),
                (int) (region2_pointB.y - region2_pointA.y));

        Mat region1 = mat.submat(REGION1_CROP);
        Mat region2 = mat.submat(REGION2_CROP);

        region1Average = Core.mean(region1).val;
        region2Average = Core.mean(region2).val;;
        percentRed1 = region1Average[0]/(region1Average[0]+region1Average[1]+region1Average[2]);
        percentRed2 = region2Average[0]/(region2Average[0]+region2Average[1]+region2Average[2]);
        percentBlue1 = region1Average[2]/(region1Average[0]+region1Average[1]+region1Average[2]);
        percentBlue2 = region2Average[2]/(region2Average[0]+region2Average[1]+region2Average[2]);

        if(rgbIndex == 0){
            if(percentRed1 > percentageThreshold){
                prediction = 'l';
            } else if(percentRed2 > percentageThreshold){
                prediction = 'c';
            } else {
                prediction = 'r';
            }
        } else if(rgbIndex == 2){
            if(percentBlue1 > percentageThreshold){
                prediction = 'l';
            } else if(percentBlue2 > percentageThreshold){
                prediction = 'c';
            } else {
                prediction = 'r';
            }
        }

//        if(region1Average[rgbIndex] > threshold){
//            prediction = 'l';
//        } else if(region2Average[rgbIndex] > threshold){
//            prediction = 'c';
//        } else {
//            prediction = 'r';
//        }
        region1.release();
        region2.release();
//        Imgproc.putText(mat, (String)prediction, new org.opencv.core.Point(100, 100), 1, 1, new org.opencv.core.Scalar(0, 0, 255), 1);
        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines
        Imgproc.rectangle(
                input, // Buffer to draw on
                region2_pointA, // First point which defines the rectangle
                region2_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

//


        return input;
    }
    public double getPercentRed1(){
        return percentRed1;
    }
    public double getPercentRed2(){
        return percentRed2;
    }
    public double getPercentBlue1(){
        return percentBlue1;
    }
    public double getPercentBlue2(){
        return percentBlue2;
    }
    public char getAnalysis() {
//        telemetry.addData("prediction: ", prediction);
        return prediction;
    }
    public double[] getRegion1Average(){
        return region1Average;
    }
    public double [] getRegion2Average(){
        return region2Average;
    }
    public void setRegionPoints(Point region1_pA, Point region1_pB, Point region2_pA, Point region2_pB){
        region1_pointA = region1_pA;
        region1_pointB = region1_pB;
        region2_pointA = region2_pA;
        region2_pointB = region2_pB;
    }
    public Point getRegion1_pointA(){
        return region1_pointA;
    }
    public Point getRegion1_pointB(){
        return region1_pointB;
    }
    public Point getRegion2_pointA(){
        return region2_pointA;
    }
    public Point getRegion2_pointB(){
        return region2_pointB;
    }

    public String frameArr(Mat input) {
        MatOfByte buffer = new MatOfByte();
        Imgcodecs.imencode(".png", input, buffer);

        return buffer.toString();
    }
    private static String getClassLabel(float[][] output) {
        // Find the index with the highest confidence
        int maxIndex = 0;
        float maxConfidence = output[0][0];

        for (int i = 1; i < NUM_CLASSES; i++) {
            if (output[0][i] > maxConfidence) {
                maxIndex = i;
                maxConfidence = output[0][i];
            }
        }

        // Map index to class label (replace with your own labels)
        String[] classLabels = {"ClassA", "ClassB", "ClassC"};
        return classLabels[maxIndex];
    }
}