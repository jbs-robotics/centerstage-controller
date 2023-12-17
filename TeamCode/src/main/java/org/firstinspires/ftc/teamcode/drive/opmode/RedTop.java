package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvCameraFactory;

//TODO: upload new version of code
/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive", name="Red Top")
public class RedTop extends LinearOpMode {
    private DcMotor lift = null;
    private OpenCvCamera webcam = null;
    private ColorDetectorPipeline pipeline = null;
    private Servo intake, lock = null;
    private int liftDelay = 1000;
    private double intakeUp = 0.75, intakeDown = 0;

    private VisionPortal visionPortal;
//    private Servo intakeServo = null;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new ColorDetectorPipeline(telemetry, hardwareMap, 0);
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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        intake = hardwareMap.get(Servo.class, "intake");
        intake.setDirection(Servo.Direction.FORWARD);
//        initTfod();
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setDirection(DcMotor.Direction.REVERSE);
        intake.setPosition(intakeUp);
//        telemetry.addData("prediction: ", pipeline.getAnalysis());
        telemetry.addData(">","Ready to start");
        waitForStart();
//        telemetryTfod();
//        telemetry.update();
//        List<Recognition> currentRecognitions = tfod.getRecognitions();
//        String TFODPrediction = currentRecognitions.get(0).getLabel();
        char TFODPrediction = pipeline.getAnalysis();
        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d(new Vector2d(60, 10), Math.toRadians(180)));

        switch(TFODPrediction) {
            case 'l': //left
                Trajectory left1 = drive.trajectoryBuilder(new Pose2d(60, 10, Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(new Vector2d(25, 5), Math.toRadians(-80)))
                        .build();
                drive.followTrajectory(left1);
                Trajectory forwardOffset = drive.trajectoryBuilder(left1.end())
                        .forward(2)
                        .build();
                drive.followTrajectory(forwardOffset);
                Trajectory lOffset1 = drive.trajectoryBuilder(forwardOffset.end())
                        .strafeRight(4)
                        .build();
                drive.followTrajectory(lOffset1);
                //place prop on spike mark
                placeOnSpike();
//                lift.setPower(1);
//                sleep(liftDelay/5);
//                lift.setPower(0);
                Trajectory left2 = drive.trajectoryBuilder(lOffset1.end())
                        .lineToLinearHeading(new Pose2d(new Vector2d(18, 50), Math.toRadians(100.5)))
                        .build();
                drive.followTrajectory(left2);
                Trajectory Loffset2 = drive.trajectoryBuilder(left2.end())
                        .forward(10)
                        .build();
                drive.turn(Math.toRadians(-30));
                drive.followTrajectory(Loffset2);
                //place pixel on canvas
//                lift.setPower(-1);
//                sleep(liftDelay/5);
//                lift.setPower(0);
//                placeOnCanvas();
//                lift.setPower(1);
//                sleep(liftDelay);
//                lift.setPower(0);
                break;
            case 'c': //center
                Trajectory center1 = drive.trajectoryBuilder(new Pose2d(60, 10, Math.toRadians(180)))
                        .forward(22)
                        .build();
                drive.followTrajectory(center1);
                Trajectory Coffset = drive.trajectoryBuilder(center1.end())
                        .strafeLeft(3)
                        .build();
                drive.followTrajectory(Coffset);
                //place prop on spike mark
                placeOnSpike();
                Trajectory center2 = drive.trajectoryBuilder(Coffset.end())
                        .lineToLinearHeading(new Pose2d(35.5, 50, Math.toRadians(90)))
                        .build();
                drive.followTrajectory(center2);
                drive.turn(Math.toRadians(-25));
                Trajectory center3 = drive.trajectoryBuilder(center2.end()).forward(4).build();
                drive.followTrajectory(center3);
                Trajectory center4 = drive.trajectoryBuilder(center3.end())
                        .strafeRight(20)
                        .build();
                drive.followTrajectory(center4);
//                Trajectory center5 = drive.trajectoryBuilder(center4.end()).forward(5).build();
//                drive.followTrajectory(center5);
                //place pixel on canvas
//                placeOnCanvas();
//                lift.setPower(1);
//                sleep(liftDelay);
//                lift.setPower(0);
                break;
            case 'r': //right
//                lift.setPower(1);
//                sleep(liftDelay/4);
//                lift.setPower(0.01);
                Trajectory right1 = drive.trajectoryBuilder(new Pose2d(60, 10, Math.toRadians(180)))
                        .strafeRight(27)
                        .build();
                Trajectory right2 = drive.trajectoryBuilder(right1.end())
                        .lineToSplineHeading(new Pose2d(30, 32, Math.toRadians(-70)))
                        .build();
                Trajectory rOffset1 = drive.trajectoryBuilder(right2.end())
                        .back(6)
                        .build();
                Trajectory rOffset2 = drive.trajectoryBuilder(rOffset1.end())
                        .strafeRight(3)
                        .build();
                Trajectory right4 = drive.trajectoryBuilder(right2.end())
                        .forward(3)
                        .build();
                Trajectory right5 = drive.trajectoryBuilder(right4.end())
                        .strafeRight(10)
                        .build();
                drive.followTrajectory(right1);
                drive.followTrajectory(right2);
                drive.followTrajectory(rOffset1);
                drive.followTrajectory(rOffset2);
                drive.followTrajectory(right4);
                drive.followTrajectory(right5);
//                drive.turn(Math.toRadians(20));
//                drive.followTrajectory(Roffset1);
                //place prop on spike mark
                placeOnSpike();
                Trajectory right3 = drive.trajectoryBuilder(right5.end())
                        .lineToSplineHeading(new Pose2d(42, 50, Math.toRadians(115)))
                        .build();
                drive.followTrajectory(right3);
                drive.turn(Math.toRadians(-40));
                Trajectory Roffset2 = drive.trajectoryBuilder(right3.end())
                        .forward(10)
                        .build();
                drive.followTrajectory(Roffset2);
                //place pixel on canvas
//                placeOnCanvas();

                break;
            default:
                telemetry.addData("wtf how", "no but actually how");
                break;
        }
    }
    private void placeOnSpike(){
        lift.setPower(1);
        sleep(liftDelay/8);
        lift.setPower(.25);
        intake.setPosition(.5);
        sleep(1000);
        intake.setPosition(.3);
//        intake.setPosition(intakeDown);
        sleep(4000);
        intake.setPosition(intakeUp);
    }
    private void placeOnCanvas(){
        lift.setPower(1);
        sleep(liftDelay/4);
        lift.setPower(0);
//        lock.setPosition(1);
        intake.setPosition(intakeUp);

    }
}
/* non-default options for tfod
        * TfodProcessor.Builder myTfodProcessorBuilder;
          TfodProcessor myTfodProcessor;
          // Create a new TFOD Processor Builder object.
          myTfodProcessorBuilder = new TfodProcessor.Builder();

          // Optional: set other custom features of the TFOD Processor (4 are shown here).
          myTfodProcessorBuilder.setMaxNumRecognitions(1);  // Max. number of recognitions the network will return
          myTfodProcessorBuilder.setUseObjectTracker(true);  // Whether to use the object tracker
          myTfodProcessorBuilder.setTrackerMaxOverlap((float) 0.2);  // Max. % of box overlapped by another box at recognition time
          myTfodProcessorBuilder.setTrackerMinSize(16);  // Min. size of object that the object tracker will track

          // Create a TFOD Processor by calling build()
          myTfodProcessor = myTfodProcessorBuilder.build();
        * */