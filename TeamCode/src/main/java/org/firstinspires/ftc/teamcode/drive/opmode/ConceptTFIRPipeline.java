package org.firstinspires.ftc.teamcode.drive.opmode;

import static java.lang.Thread.sleep;

import android.graphics.Bitmap;

import org.firstinspires.ftc.teamcode.ml.Blue;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ml.BlueQ;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;
import android.graphics.BitmapFactory;
import android.media.ThumbnailUtils;

import org.tensorflow.lite.DataType;
import org.tensorflow.lite.support.tensorbuffer.TensorBuffer;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;




public class ConceptTFIRPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    static HardwareMap hardwareMap;
    Mat mat = new Mat();
    private String filepath = "blue.tflite";
    private static int NUM_CLASSES = 3;
    private int rgbIndex = 2;

    private int width = 40, height = 40;

    public ConceptTFIRPipeline(Telemetry telemetry, HardwareMap hardwareMap, int rgbI) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.rgbIndex = rgbI;
    }

    public ConceptTFIRPipeline(Telemetry telemetry, HardwareMap hardwareMap, int rgbI, int w, int h) {
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

    private char prediction = 'c';
    private static final int imageSize = 224;

    @Override
    public Mat processFrame(Mat input) {
        mat = input;

        // Convert Mat to byte array
        Bitmap image = Bitmap.createBitmap(mat.width(), mat.height(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(mat, image);

        // Convert to square image recognizable by model
        int dimension = Math.min(image.getWidth(), image.getHeight());
        image = ThumbnailUtils.extractThumbnail(image, dimension, dimension);

        image = Bitmap.createScaledBitmap(image, imageSize, imageSize, false);
        prediction = classifyImage(image);

        return input;
    }

    public char getAnalysis() {
        return prediction;
    }

    public char classifyImage(Bitmap image) {
        try {
            BlueQ model = BlueQ.newInstance(hardwareMap.appContext);

            // Creates inputs for reference.
            TensorBuffer inputFeature0 = TensorBuffer.createFixedSize(new int[]{1, 224, 224, 3}, DataType.FLOAT32);

            // Allocate ByteBuffer for TF processing
            ByteBuffer byteBuffer = ByteBuffer.allocateDirect(4 * imageSize * imageSize * 3);
            byteBuffer.order(ByteOrder.nativeOrder());

            // Convert Bitmap to int array
            int[] intValues = new int[imageSize * imageSize];
            image.getPixels(intValues, 0, image.getWidth(), 0, 0, image.getWidth(), image.getHeight());

            // Dump int array into byteBuffer
            int currentPixel = 0;
            for (int i = 0; i < imageSize; i++) {
                for (int j = 0; j < imageSize; j++) {
                    int val = intValues[currentPixel];
                    // Parse pixel into R G and B and put into byteBuffer
                    // (replace 1 with 255 if TFLite does not preprocess
                    byteBuffer.putFloat(((val >> 16) & 0xFF) * (1.f / 1));
                    byteBuffer.putFloat(((val >> 8) & 0xFF) * (1.f / 1));
                    byteBuffer.putFloat((val & 0xFF) * (1.f / 1));
                    currentPixel++;
                }
            }

            inputFeature0.loadBuffer(byteBuffer);

            // Runs model inference and gets result.
            BlueQ.Outputs outputs = model.process(inputFeature0);
            TensorBuffer outputFeature0 = outputs.getOutputFeature0AsTensorBuffer();

            // Find position of most confident value
            float[] confidences = outputFeature0.getFloatArray();
            int maxPos = 0;
            float maxConfidence = 0;
            for (int i = 0; i < confidences.length; i++) {
                if (confidences[i] > maxConfidence) {
                    maxConfidence = confidences[i];
                    maxPos = i;
                }
            }
            char[] classes = {'l', 'c', 'r'};

            // Releases model resources if no longer used.
            model.close();

            return classes[maxPos];
        } catch (IOException e) {
            telemetry.addData("Error running TFLite", "an unknown error occurred running the TFLite model");
            telemetry.update();
            return 'e';
        }
    }
}