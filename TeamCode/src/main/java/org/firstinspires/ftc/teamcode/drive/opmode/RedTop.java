package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
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
    private DcMotor leftFront, leftBack, rightFront, rightBack, lift;
    private DistanceSensor distanceSensor = null;
    private OpenCvCamera webcam = null;
    private ColorDetectorPipeline pipeline = null;
    private Servo intake = null,  claw = null, fingerer = null;
    private CRServo angleServo = null;
    private int liftDelay = 1000;
    private double intakeUp = 0.7, intakeDown = 0, clawUp = 0.5, clawDown = 0.4, angleServoUp = .48, angleServoDown = 0.43;

    @Override
    public void runOpMode() throws InterruptedException {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        // Movement Motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Set Motor Direction
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);


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
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
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
                TrajectorySequence toSpikeLeft = drive.trajectorySequenceBuilder(new Pose2d(60, 10, Math.toRadians(180)))
                        .strafeRight(5)
                        .lineToLinearHeading(new Pose2d(new Vector2d(30, 7), Math.toRadians(-90)))
                        .build();
                drive.followTrajectorySequence(toSpikeLeft);
                //place prop on spike mark
                placeOnSpike();
                TrajectorySequence toBackdropLeft = drive.trajectorySequenceBuilder(toSpikeLeft.end())
                        .back(5)
                        .lineToSplineHeading(new Pose2d(22, 45, Math.toRadians(-90)))
                        .build();
                drive.followTrajectorySequence(toBackdropLeft);
                //place pixel on canvas
                placeOnCanvas();
                break;
            case 'c': //center
                TrajectorySequence toSpikeCenter = drive.trajectorySequenceBuilder(new Pose2d(60, 10, Math.toRadians(180)))
                        .forward(29)
                        .strafeLeft(3)
                        .build();
                drive.followTrajectorySequence(toSpikeCenter);
                //place prop on spike mark
                placeOnSpike();
                TrajectorySequence toBackdropCenter = drive.trajectorySequenceBuilder(toSpikeCenter.end())
                        .back(5)
                        .lineToSplineHeading(new Pose2d(31, 45, Math.toRadians(-90)))
                        .build();
                drive.followTrajectorySequence(toBackdropCenter);
                //place pixel on canvas
                placeOnCanvas();
                break;
            case 'r': //right
                TrajectorySequence toSpikeRight = drive.trajectorySequenceBuilder(new Pose2d(60, 10, Math.toRadians(180)))
                        .strafeRight(25)
                        .lineToSplineHeading(new Pose2d(30, 32, Math.toRadians(-90)))
                        .build();
                drive.followTrajectorySequence(toSpikeRight);
                //place prop on spike mark
                placeOnSpike();
                TrajectorySequence toBackdropRight = drive.trajectorySequenceBuilder(toSpikeRight.end())
                        .back(5)
                        .lineToSplineHeading(new Pose2d(41, 45, Math.toRadians(-90)))
                        .build();
                drive.followTrajectorySequence(toBackdropRight);
                //place pixel on canvas
                placeOnCanvas();
                break;
            default:
                telemetry.addData("wtf how", "no but actually how");
                break;
        }
    }
    private void placeOnSpike(){
        fingerer.setPosition(0);
        sleep(2000);
    }
    private void placeOnCanvas(){
        while(distanceSensor.getDistance(DistanceUnit.INCH) > 1){
            leftFront.setPower(-.1);
            rightFront.setPower(-.1);
            leftBack.setPower(-.1);
            rightBack.setPower(-.1);
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        angleServo.setPower(-.2);
        sleep(2200);
        angleServo.setPower(0);
        claw.setPosition(clawUp);
        sleep(1000);
    }
    private void moveBack(){

    }
}
