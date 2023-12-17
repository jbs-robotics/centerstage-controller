package org.firstinspires.ftc.teamcode.drive.opmode;

import android.content.res.AssetFileDescriptor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;

@Autonomous(name = "Testing Auto")
public class AutoTesting extends LinearOpMode {
    private DcMotor lift = null;
    private OpenCvCamera webcam = null;
    private ColorDetectorPipeline pipeline = null;
    private int liftDelay = 1500;
    private static int NUM_CLASSES=3;
    private float[][] output = new float[1][NUM_CLASSES];

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
    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new ColorDetectorPipeline(telemetry, hardwareMap, 0, 40, 40);

        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);

            }
            @Override
            public void onError(int errorCode) { telemetry.addData("Error", errorCode); }
        });

        webcam.resumeViewport();
        telemetry.addData("region1Average", pipeline.getRegion1Average()[0]);
        telemetry.addData("region2Average", pipeline.getRegion2Average()[0]);
        waitForStart();
//        try {
//            tflite = loadModel("blue.tflite");
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
//        tflite.run(webcam.getFrameBitmap(Continuation.create(Thread.currentThread(), new ContinuationResultConsumer<Bitmap>() {
//            @Override
//            public void accept(Bitmap bitmap) {
//                Mat tmp = new Mat();
//                Imgproc.resize(input, tmp, new Size(224, 224));
//                Bitmap bm = Bitmap.createBitmap(tmp.cols(), tmp.rows(), Bitmap.Config.ARGB_8888);
//
//                tflite.run(bm, output);
//                return input;
//            }
//        })), output);

//        Telemetry telemetry = new Telemetry();
//        telemetry.addData("Inference Result", "Class: " +);
        telemetry.update();
        while(opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("Frame Count", webcam);
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("region1Average", pipeline.getRegion1Average()[0]);
            telemetry.addData("region1Average", pipeline.getRegion1Average()[1]);
            telemetry.addData("region1Average", pipeline.getRegion1Average()[2]);
            telemetry.addData("region2Average", pipeline.getRegion2Average()[0]);
            telemetry.addData("region2Average", pipeline.getRegion2Average()[1]);
            telemetry.addData("region2Average", pipeline.getRegion2Average()[2]);
            telemetry.addData("percentRed1", pipeline.getPercentRed1());
            telemetry.addData("percentRed2", pipeline.getPercentRed2());
            telemetry.addData("percentBlue1", pipeline.getPercentBlue1());
            telemetry.addData("percentBlue2", pipeline.getPercentBlue2());
            telemetry.addData("Prediction", pipeline.getAnalysis());
            //display the frame
//            telemetry.addData("TF Lite", pipeline.getTfliteModel());
//            telemetry.addData("Inference Result", "Class: " + pipeline.getAnalysis());
//            double confidence[] = pipeline.getConfidence();
//            telemetry.addData("Confidence", "l: " + confidence[0] + " r: " + confidence[1] + " c: " + confidence[2]);
//            telemetry.addData("Confidence", pipeline.getConfidence());
//            telemetry.addData("Frame", pipeline.frameArr(pipeline.processFrame()));
            telemetry.update();
        }
//        lift.setPower(-1);
//        sleep(liftDelay);
//        lift.setPower(0);
    }
    private MappedByteBuffer loadModelFile(String filename) throws IOException {
        AssetFileDescriptor fileDescriptor = hardwareMap.appContext.getAssets().openFd(filename);
        FileInputStream inputStream = new FileInputStream(fileDescriptor.getFileDescriptor());
        FileChannel fileChannel = inputStream.getChannel();
        long startOffset = fileDescriptor.getStartOffset();
        long declaredLength = fileDescriptor.getDeclaredLength();
        return fileChannel.map(FileChannel.MapMode.READ_ONLY, startOffset, declaredLength);
    }
}
