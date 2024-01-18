package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
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
    private double intakeUp = 0.75, intakeDown = 0, clawUp = 0.5, clawDown = 0.4;

    private OpenCvCamera webcam = null;
    private ColorDetectorPipeline pipeline = null;
    private Servo intake, claw = null, fingerer;
    private CRServo angleServo = null;

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
        angleServo = hardwareMap.get(CRServo.class, "angleServo");
        claw = hardwareMap.get(Servo.class, "claw");
        fingerer = hardwareMap.get(Servo.class, "fingerer2");
        lift.setDirection(DcMotor.Direction.REVERSE);
        intake.setPosition(intakeUp);
        claw.setPosition(clawDown);
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
                TrajectorySequence toSpikeLeft = drive.trajectorySequenceBuilder(new Pose2d(60, -37, Math.toRadians(180)))
                        .lineToSplineHeading(new Pose2d(20, -37, Math.toRadians(-90)))
                        .build();
                drive.followTrajectorySequence(toSpikeLeft);
                //place pixel on spike mark
                placeOnSpike();
                TrajectorySequence toBackdropLeft = drive.trajectorySequenceBuilder(toSpikeLeft.end())
                        .back(3)
                        .strafeRight(20)
                        .back(20)
                        .splineToConstantHeading(new Vector2d(26, 38), Math.toRadians(0))
                        .back(14)
                        .build();
                drive.followTrajectorySequence(toBackdropLeft);
                //place pixel on canvas
                placeOnCanvas();
                break;
            case 'c': //center
                TrajectorySequence toSpikeCenter = drive.trajectorySequenceBuilder(new Pose2d(60, -37, Math.toRadians(180)))
                        .forward(28)
                        .strafeRight(3)
                        .build();
                drive.followTrajectorySequence(toSpikeCenter);

                //place pixel on spike mark
                placeOnSpike();

                TrajectorySequence toBackdropCenter = drive.trajectorySequenceBuilder(toSpikeCenter.end())
                        .back(10)
                        .strafeLeft(22)
                        .forward(24)
                        .splineToLinearHeading(new Pose2d(10, -30, Math.toRadians(-90)), Math.toRadians(90))
                        .back(20)
                        .splineToConstantHeading(new Vector2d(40, 38), Math.toRadians(0))
                        .back(17)
                        .build();
                drive.followTrajectorySequence(toBackdropCenter);

                //place pixel on canvas
                placeOnCanvas();
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
        fingerer.setPosition(0);
        sleep(500);
    }
    private void placeOnCanvas(){
        angleServo.setPower(-.2);
        sleep(2000);
        angleServo.setPower(0);
        claw.setPosition(clawUp);
        sleep(1000);
    }
}