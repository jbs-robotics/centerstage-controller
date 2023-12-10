package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.content.res.AssetFileDescriptor;
import android.graphics.Bitmap;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.tensorflow.lite.Interpreter;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;

public class Pipeline extends OpenCvPipeline {
    private Interpreter tflite;
    private String filepath = "blue.tflite";
    private static int NUM_CLASSES=3;
    private float[][] output = new float[1][NUM_CLASSES];
    public static Interpreter loadModel(String modelPath) throws IOException {
        File modelFile = new File(modelPath);
        MappedByteBuffer modelBuffer = new FileInputStream(modelFile).getChannel().map(FileChannel.MapMode.READ_ONLY, 0, modelFile.length());

        Interpreter.Options options = new Interpreter.Options();
        Interpreter tfliteInterpreter = new Interpreter(modelBuffer, options);

        return tfliteInterpreter;
    }
    private MappedByteBuffer loadModelFile(String filename) throws IOException {
        AssetFileDescriptor fileDescriptor = hardwareMap.appContext.getAssets().openFd(filename);
        FileInputStream inputStream = new FileInputStream(fileDescriptor.getFileDescriptor());
        FileChannel fileChannel = inputStream.getChannel();
        long startOffset = fileDescriptor.getStartOffset();
        long declaredLength = fileDescriptor.getDeclaredLength();
        return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength);
    }
    @Override
    public void init(Mat frame){
        try {
//            MappedByteBuffer tfliteModel = FileUtil.loadMappedFile(hardwareMap.appContext, filepath);
            tflite = loadModel(filepath);
            if (tflite == null) {
                telemetry.addData("Null", "Null");
                telemetry.update();
                throw new IOException();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    @Override
    public Mat processFrame(Mat input) {
        Mat tmp = new Mat();
        Imgproc.resize(input, tmp, new Size(224, 224));
        Bitmap bm = Bitmap.createBitmap(tmp.cols(), tmp.rows(), Bitmap.Config.ARGB_8888);

        tflite.run(bm, output);
        return input;
    }

    public char getAnalysis() {
        char[] analysis = {'l', 'c', 'r'};
        char result = analysis[1];
        float confidence = output[0][0];
        for (int i = 1; i < NUM_CLASSES; i++) {
            if (output[0][i] > confidence) {
                telemetry.addData("Confidence", output[0][i]);
//                telemetry.update();
                result = analysis[i];
                confidence = output[0][i];
            }
        }
        return result;
    }
    public double[] getConfidence() {
        double[] confidence = new double[NUM_CLASSES];
        for (int i = 0; i < NUM_CLASSES; i++) {
            confidence[i] = output[0][i];
        }
        return confidence;
    }
    public String getTfliteModel() {
        if(tflite == null)
            return "Null";
        else
            return tflite.toString();
//        return filepath;
    }
}

/*
    public static Interpreter loadModel(String modelPath) throws IOException {
        File modelFile = new File(modelPath);
        MappedByteBuffer modelBuffer = new FileInputStream(modelFile).getChannel().map(FileChannel.MapMode.READ_ONLY, 0, modelFile.length());

        Interpreter.Options options = new Interpreter.Options();
        Interpreter tfliteInterpreter = new Interpreter(modelBuffer, options);

        return tfliteInterpreter;
    }

    public static void main(String[] args) {
        try {
            String modelPath = "path/to/your/thingamabob.tflite";
            Interpreter tfliteInterpreter = loadModel(modelPath);

            // Your model is now loaded, and you can perform inference using tfliteInterpreter.

        } catch (IOException e) {
            e.printStackTrace();
        }
    }*/