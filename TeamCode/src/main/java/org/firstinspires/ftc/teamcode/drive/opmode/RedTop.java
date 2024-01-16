package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvCameraFactory;

//TODO: upload new version of code
/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive", name="Red Top", preselectTeleOp="Basic: Linear OpMode")
public class RedTop extends LinearOpMode {
    private DcMotor lift = null;
    private OpenCvCamera webcam = null;
    private ColorDetectorPipeline pipeline = null;
    private Servo intake = null,  claw = null, fingerer = null;
    private CRServo angleServo = null;
    private int liftDelay = 1000;
    private double intakeUp = 0.7, intakeDown = 0, clawUp = 0.5, clawDown = 0.4, angleServoUp = .48, angleServoDown = 0.43;

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
        pipeline.setRegionPoints(new Point(0, 140), new Point(40, 180), pipeline.getRegion2_pointA(), pipeline.getRegion2_pointB());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        intake = hardwareMap.get(Servo.class, "intake");
        intake.setDirection(Servo.Direction.FORWARD);
        angleServo = hardwareMap.get(CRServo.class, "angleServo");
        claw = hardwareMap.get(Servo.class, "claw");
        lift = hardwareMap.get(DcMotor.class, "lift");
        fingerer = hardwareMap.get(Servo.class, "fingerer");
        lift.setDirection(DcMotor.Direction.REVERSE);
        intake.setPosition(intakeUp);
        claw.setPosition(clawDown);
        fingerer.setPosition(.15);

        telemetry.addData("fingerer Pos: ", fingerer.getPosition());
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
                        .forward(18)
                        .build();
                Trajectory left1_3 = drive.trajectoryBuilder(left1_2.end())
                        .back(10)
                        .build();
                Trajectory left1_4 = drive.trajectoryBuilder(left1_3.end())
                    .strafeRight(4)
                    .build();
                drive.followTrajectory(left1_2);
                drive.followTrajectory(left1_3);
                drive.followTrajectory(left1_4);
                //place prop on spike mark
                placeOnSpike();
                Trajectory left1_5 = drive.trajectoryBuilder(left1_4.end())
                        .back(12)
                        .build();
                drive.followTrajectory(left1_5);
                Trajectory left2 = drive.trajectoryBuilder(left1_5.end())
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
//                Trajectory left3_5 = drive.trajectoryBuilder(left3.end())
//                        .back(1)
//                        .build();
//                drive.followTrajectory(left3_5);
//                Trajectory left4 = drive.trajectoryBuilder(left3_5.end())
//                        .strafeLeft(10)
//                        .build();
//                drive.followTrajectory(left4);
                break;
            case 'c': //center
                Trajectory center1 = drive.trajectoryBuilder(new Pose2d(60, 10, Math.toRadians(180)))
                        .forward(38)
                        .build();
                drive.followTrajectory(center1);
                Trajectory center1_2 = drive.trajectoryBuilder(center1.end())
                        .back(11)
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
                        .lineToConstantHeading(new Vector2d(32.5, 53))
                        .build();
                drive.turn(Math.toRadians(180));
                drive.followTrajectory(center3);
                //place pixel on canvas
                placeOnCanvas();
//                Trajectory center3_5 = drive.trajectoryBuilder(center3.end())
//                        .back(1)
//                        .build();
//                drive.followTrajectory(center3_5);
//                Trajectory center4 = drive.trajectoryBuilder(center3_5.end())
//                        .strafeLeft(20)
//                        .build();
//                drive.followTrajectory(center4);
                break;
            case 'r': //right
                Trajectory right1 = drive.trajectoryBuilder(new Pose2d(60, 10, Math.toRadians(180)))
                        .strafeRight(27)
                        .build();
                Trajectory right2 = drive.trajectoryBuilder(right1.end())
                        .lineToSplineHeading(new Pose2d(30, 39, Math.toRadians(-90)))
                        .build();
                drive.followTrajectory(right1);
                drive.followTrajectory(right2);
                Trajectory right2_1 = drive.trajectoryBuilder(right2.end())
                        .forward(24)
                        .build();
                Trajectory right2_2 = drive.trajectoryBuilder(right2_1.end())
                        .back(18)
                        .build();
                drive.followTrajectory(right2_1);
                drive.followTrajectory(right2_2);
                //place prop on spike mark
                placeOnSpike();
                Trajectory right2_3 = drive.trajectoryBuilder(right2_2.end())
                        .back(12)
                        .build();
                drive.followTrajectory(right2_3);
                Trajectory right3 = drive.trajectoryBuilder(right2_3.end())
                        .lineToSplineHeading(new Pose2d(42, 50, Math.toRadians(90)))
                        .build();
                drive.followTrajectory(right3);
                Trajectory right4 = drive.trajectoryBuilder(right3.end().plus(new Pose2d(0, 0, Math.toRadians(180))))
                        .lineToConstantHeading(new Vector2d(39, 54))
                        .build();
                drive.turn(Math.toRadians(180));
                drive.followTrajectory(right4);
                //place pixel on canvas
                placeOnCanvas();
//                Trajectory right4_5 = drive.trajectoryBuilder(right4.end())
//                        .back(1)
//                        .build();
//                drive.followTrajectory(right4_5);
//                Trajectory right5 = drive.trajectoryBuilder(right4_5.end())
//                        .strafeLeft(24)
//                        .build();
//                drive.followTrajectory(right5);

                break;
            default:
                telemetry.addData("wtf how", "no but actually how");
                break;
        }
    }
    private void placeOnSpike(){
//        sleep(1000);
//        lift.setPower(1);
//        sleep((int)(liftDelay/6.4));
//        lift.setPower(.25);
//
//        double intakePos = .7;
//        for(int i = 0; i < 70; i++){
//            intake.setPosition(intakePos);
//            intakePos -= .01;
//            sleep(100);
//        }
//
//        intake.setPosition(0);
////        intake.setPosition(intakeDown);
////        sleep(4000);
//        intake.setPosition(intakeUp);
        fingerer.setPosition(0);
        sleep(2000);

    }
    private void placeOnCanvas(){
        angleServo.setPower(-.2);
        sleep(2000);
        angleServo.setPower(0);
//        sleep(1000);
        claw.setPosition(clawUp);
        sleep(1000);

//        angleServo.setPosition(angleServoUp);
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