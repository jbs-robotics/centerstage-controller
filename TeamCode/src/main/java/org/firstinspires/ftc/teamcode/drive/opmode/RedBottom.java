package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive", name="Red Bottom", preselectTeleOp="Basic: Linear OpMode")
public class RedBottom extends LinearOpMode {
    private DcMotor lift = null;
    private int liftDelay = 1000;
    private double intakeUp = 0.75, intakeDown = 0;

    private OpenCvCamera webcam = null;
    private ColorDetectorPipeline pipeline = null;
    private Servo intake, lock = null;

    TfodProcessor tfod;
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
        drive.setPoseEstimate(new Pose2d(new Vector2d(60, -37), Math.toRadians(180)));
        switch(TFODPrediction){
            case 'l': //left
                Trajectory left1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(35, -37, Math.toRadians(-90)))
                        .build();
                drive.followTrajectory(left1);

                //place prop on spike mark
                Trajectory left2 = drive.trajectoryBuilder(left1.end())
                        .splineToConstantHeading(new Vector2d(35, -33), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(50, -37, Math.toRadians(90)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(55, -12), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(44.5, -12), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(28, 50), Math.toRadians(180))
                        .build();
                drive.followTrajectory(left2);
                drive.turn(Math.toRadians(180));
                Trajectory left3 = drive.trajectoryBuilder(left2.end())
                        .lineToConstantHeading(new Vector2d(28, 53))
                        .build();
                drive.followTrajectory(left3);
                //place pixel on canvas
                break;
            case 'c': //center
                Trajectory center1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .forward(27)
                        .build();
                drive.followTrajectory(center1);

                //place pixel on spike mark
                placeOnSpike();
                Trajectory center2 = drive.trajectoryBuilder(center1.end())
                        .splineToConstantHeading(new Vector2d(35, -37), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(50, -37, Math.toRadians(90)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(55, -12), Math.toRadians(180))
                        .splineToConstantHeading(new Vector2d(35.5, 50), Math.toRadians(90))
                        .build();
                drive.followTrajectory(center2);
                drive.turn(Math.toRadians(180));
                Trajectory center3 = drive.trajectoryBuilder(center2.end().plus(new Pose2d(0, 0, Math.toRadians(180))))
                        .lineToConstantHeading(new Vector2d(35.5, 53))
                        .build();
                drive.followTrajectory(center3);
                //place pixel on canvas
                break;
            case 'r': //right
                Trajectory right1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToSplineHeading(new Pose2d(35, -33, Math.toRadians(90)))
                        .build();
                Trajectory right2 = drive.trajectoryBuilder(right1.end())
                        .splineToSplineHeading(new Pose2d(53, -37, Math.toRadians(90)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(55, -12), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(48, -12), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(42, 50), Math.toRadians(90))
                        .build();
                drive.followTrajectory(right1);
                //place pixel on spike mark
                drive.followTrajectory(right2);
                drive.turn(Math.toRadians(180));
                Trajectory right3 = drive.trajectoryBuilder(right2.end().plus(new Pose2d(0, 0, Math.toRadians(180))))
                        .lineToConstantHeading(new Vector2d(42, 53))
                        .build();
                drive.followTrajectory(right3);

                //place pixel on canvas
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