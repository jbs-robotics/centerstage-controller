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
    private Servo intake = null, angleServo = null, claw = null;
    private int liftDelay = 1000;
    private double intakeUp = 0.7, intakeDown = 0, clawUp = 0.5, clawDown = 0.4, angleServoUp = .5, angleServoDown = 0.43;

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
        angleServo = hardwareMap.get(Servo.class, "angleServo");
        claw = hardwareMap.get(Servo.class, "claw");
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setDirection(DcMotor.Direction.REVERSE);
        intake.setPosition(intakeUp);
        claw.setPosition(clawDown);


        telemetry.addData(">","Ready to start");
        waitForStart();
        char TFODPrediction = pipeline.getAnalysis();
        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d(new Vector2d(60, 10), Math.toRadians(180)));

        switch(TFODPrediction) {
            case 'l': //left
                Trajectory left1 = drive.trajectoryBuilder(new Pose2d(60, 10, Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(new Vector2d(26, 14), Math.toRadians(-90)))
                        .build();
                drive.followTrajectory(left1);
                Trajectory left1_2_1 = drive.trajectoryBuilder(left1.end())
                        .strafeLeft(3)
                        .build();
                Trajectory left1_2 = drive.trajectoryBuilder(left1_2_1.end())
                        .forward(12)
                        .build();
                Trajectory left1_3 = drive.trajectoryBuilder(left1_2.end())
                        .back(12)
                        .build();
                drive.followTrajectory(left1_2);
                drive.followTrajectory(left1_3);
                //place prop on spike mark
                placeOnSpike();
                Trajectory left2 = drive.trajectoryBuilder(left1_3.end())
                        .lineToLinearHeading(new Pose2d(new Vector2d(18, 50), Math.toRadians(90)))
                        .build();
                drive.followTrajectory(left2);
                Trajectory left3 = drive.trajectoryBuilder(left2.end().plus(new Pose2d(0, 0, Math.toRadians(180))))
                        .lineToConstantHeading(new Vector2d(26, 54))
                        .build();
                drive.turn(Math.toRadians(180));
                drive.followTrajectory(left3);
                //place pixel on canvas
                placeOnCanvas();
                break;
            case 'c': //center
                Trajectory center1 = drive.trajectoryBuilder(new Pose2d(60, 10, Math.toRadians(180)))
                        .forward(35)
                        .build();
                drive.followTrajectory(center1);
                Trajectory center1_2 = drive.trajectoryBuilder(center1.end())
                        .back(12)
                        .build();
                drive.followTrajectory(center1_2);
                //place prop on spike mark
                placeOnSpike();
                Trajectory center1_5 = drive.trajectoryBuilder(center1_2.end())
                        .back(5)
                        .build();
                Trajectory center2 = drive.trajectoryBuilder(center1_5.end())
                        .lineToLinearHeading(new Pose2d(35.5, 50, Math.toRadians(90)))
                        .build();
                drive.followTrajectory(center1_5);
                drive.followTrajectory(center2);

                Trajectory center3 = drive.trajectoryBuilder(center2.end().plus(new Pose2d(0, 0, Math.toRadians(180))))
                        .lineToConstantHeading(new Vector2d(36, 53.5))
                        .build();
                drive.turn(Math.toRadians(180));
                drive.followTrajectory(center3);
                //place pixel on canvas
                placeOnCanvas();
                break;
            case 'r': //right
                Trajectory right1 = drive.trajectoryBuilder(new Pose2d(60, 10, Math.toRadians(180)))
                        .strafeRight(27)
                        .build();
                Trajectory right2 = drive.trajectoryBuilder(right1.end())
                        .lineToSplineHeading(new Pose2d(30, 36, Math.toRadians(-90)))
                        .build();
                drive.followTrajectory(right1);
                drive.followTrajectory(right2);


                //place prop on spike mark
                placeOnSpike();

                Trajectory right3 = drive.trajectoryBuilder(right2.end())
                        .lineToSplineHeading(new Pose2d(42, 50, Math.toRadians(90)))
                        .build();
                drive.followTrajectory(right3);
                Trajectory right4 = drive.trajectoryBuilder(right3.end().plus(new Pose2d(0, 0, Math.toRadians(180))))
                        .lineToConstantHeading(new Vector2d(42, 54))
                        .build();
                drive.turn(Math.toRadians(180));
                drive.followTrajectory(right4);
                //place pixel on canvas
                placeOnCanvas();

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
        intake.setPosition(.2);
//        intake.setPosition(intakeDown);
        sleep(4000);
        intake.setPosition(intakeUp);
    }
    private void placeOnCanvas(){
        angleServo.setPosition(angleServoDown);
        sleep(3500);
        claw.setPosition(clawUp);
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