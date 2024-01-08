package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive", name="Blue Top")
public class BlueTop extends LinearOpMode {
    private DcMotor lift = null;
    private int liftDelay = 1000;
    private double intakeUp = 0.75, intakeDown = 0;

    private OpenCvCamera webcam = null;
    private ColorDetectorPipeline pipeline = null;
    private Servo intake, lock = null;
    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new ColorDetectorPipeline(telemetry, hardwareMap, 2);
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

        pipeline.setRegionPoints(new Point(40, 140), new Point(80, 180), pipeline.getRegion2_pointA(), pipeline.getRegion2_pointB());
        telemetry.addData("region1_pointA: ", pipeline.getRegion1_pointA());
        telemetry.addData("region1_pointB: ", pipeline.getRegion1_pointB());
        telemetry.addData("region2_pointA: ", pipeline.getRegion2_pointA());
        telemetry.addData("region2_pointB: ", pipeline.getRegion2_pointB());
        webcam.resumeViewport();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        intake = hardwareMap.get(Servo.class, "intake");
//        lock = hardwareMap.get(Servo.class, "lock");
        intake.setDirection(Servo.Direction.FORWARD);
//        lock.setDirection(Servo.Direction.FORWARD);
//        initTfod();
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setDirection(DcMotor.Direction.REVERSE);
        intake.setPosition(intakeUp);
        telemetry.addData(">", "Press Play to start op mode");

        waitForStart();
        char TFODPrediction = pipeline.getAnalysis();
//        telemetryTfod();
//        telemetry.update();
//        List<Recognition> currentRecognitions = tfod.getRecognitions();
//        String TFODPrediction = currentRecognitions.get(0).getLabel();
//        String TFODPrediction = "c";
        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d(new Vector2d(-60, 10), Math.toRadians(0)));

        switch(TFODPrediction) {
            case 'r': //right
                Trajectory right1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(new Vector2d(-25, 5), Math.toRadians(-90)))
                        .build();
                drive.followTrajectory(right1);
                //place prop on spike mark
                placeOnSpike();

                Trajectory right2 = drive.trajectoryBuilder(right1.end())
                        .lineToLinearHeading(new Pose2d(new Vector2d(-18, 50), Math.toRadians(90)))
                        .build();
                drive.followTrajectory(right2);
                //place pixel on canvas
//                placeOnCanvas();
//                lift.setPower(1);
//                sleep(liftDelay);
//                lift.setPower(0);
                break;
            case 'c': //center
                Trajectory center1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .forward(22)
                        .build();
                drive.followTrajectory(center1);

                //place prop on spike mark
                placeOnSpike();
                Trajectory center2 = drive.trajectoryBuilder(center1.end())
                        .lineToLinearHeading(new Pose2d(-35.5, 50, Math.toRadians(90)))
                        .build();
                drive.followTrajectory(center2);
                //place pixel on canvas
//                placeOnCanvas();
//                lift.setPower(1);
//                sleep(liftDelay);
//                lift.setPower(0);
                break;
            case 'l': //left
                Trajectory left1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .strafeLeft(27)
                        .build();
                Trajectory left2 = drive.trajectoryBuilder(left1.end())
                        .lineToSplineHeading(new Pose2d(-30, 32, Math.toRadians(-90)))
                        .build();
//                Trajectory lOffset2 = drive.trajectoryBuilder(rOffset1.end())
//                        .strafeLeft(10)
//                        .build();
                drive.followTrajectory(left1);
                drive.followTrajectory(left2);
                //place prop on spike mark
                placeOnSpike();
                Trajectory left5 = drive.trajectoryBuilder(left2.end())
                        .lineToSplineHeading(new Pose2d(-42, 50, Math.toRadians(115)))
                        .build();
                drive.followTrajectory(left5);
                //place pixel on canvas
//                placeOnCanvas();
//                lift.setPower(1);
//                sleep(liftDelay);
//                lift.setPower(0);
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
