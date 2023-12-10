package org.firstinspires.ftc.teamcode.drive.opmode;

import android.content.res.AssetFileDescriptor;
import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.tensorflow.lite.Interpreter;
import org.tensorflow.lite.support.common.FileUtil;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;

public class TfrecPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    static HardwareMap hardwareMap;
    Mat mat = new Mat();
    private Interpreter tflite;
    private String filepath = "blue_v2.tflite";
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
    public static Interpreter loadModel(String modelPath) throws IOException {
        try {
            MappedByteBuffer modelFile = FileUtil.loadMappedFile(hardwareMap.appContext, modelPath);
            return new Interpreter(modelFile, new Interpreter.Options());
        }
        catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public TfrecPipeline(Telemetry telemetry, HardwareMap hardwareMap, int rgbI) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.rgbIndex = rgbI;
    }
    @Override
    public void init(Mat frame) {
        telemetry.addData("test", "test");
        telemetry.update();

        tflite = new Interpreter(new File(filepath));

//        MappedByteBuffer tfliteModel = null;
//        try {
//            tfliteModel = FileUtil.loadMappedFile(hardwareMap.appContext, filepath);
//            telemetry.addData("file loaded", "file loaded");
//            telemetry.update();
//            if (tflite == null) {
//                telemetry.addData("Null", "Null");
//                telemetry.update();
//                throw new IOException();
//            }
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
//        try {
//            tflite = new Interpreter(tfliteModel, new Interpreter.Options());
//        } catch (Exception e) {
//            e.printStackTrace();
//
//        }
    }
    final Scalar BLUE = new Scalar(0, 0, 255);
    private double[] region1Average = new double[3];
    private double[] region2Average = new double[3];
    private char predictionLabel = 'c';
    private String prediction = "c";
    private double threshold = 180;
    @Override
    public Mat processFrame(Mat input) {
        mat = input;
        bitmap = Bitmap.createBitmap(input.cols(), input.rows(), Bitmap.Config.ARGB_8888);
        tflite.run(bitmap, output);
        prediction = getClassLabel(output);
        return input;
    }
    public char getAnalysis() {
//        telemetry.addData("prediction: ", prediction);
        return predictionLabel;
    }
    public double[] getRegion1Average(){
        return region1Average;
    }
    public double [] getRegion2Average(){
        return region2Average;
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