package org.firstinspires.ftc.teamcode.drive.opmode;

import android.content.res.AssetFileDescriptor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.calib3d.StereoBM;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;

@Autonomous(name = "Testing 2 Camera Auto")
public class DualCameraTesting extends LinearOpMode {
    private OpenCvCamera webcam1 = null, webcam2 = null;
    private DualCameraPipeline pipeline = null;
    private int liftDelay = 1500;
    private static int NUM_CLASSES=3;
    private Mat left = null, right = null;
    private StereoBM stereo = null;
    @Override
    public void runOpMode() {
        stereo = StereoBM.create();
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new DualCameraPipeline(telemetry, hardwareMap, 0);
        webcam1.setPipeline(pipeline);
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam1.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("webcam1", "started");
            }
            @Override
            public void onError(int errorCode) { telemetry.addData("Error", errorCode); }
        });
        webcam1.resumeViewport();

        webcam2 = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        pipeline = new DualCameraPipeline(telemetry, hardwareMap, 1);
        webcam2.setPipeline(pipeline);
        webcam2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam2.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("webcam2", "started");
            }
            @Override
            public void onError(int errorCode) { telemetry.addData("Error", errorCode); }
        });
        webcam2.resumeViewport();

        waitForStart();
        telemetry.update();
        while(opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("Frame Count", webcam1.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam1.getFps()));
            telemetry.addData("Total frame time ms", webcam1.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam1.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam1.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam1.getCurrentPipelineMaxFps());
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
