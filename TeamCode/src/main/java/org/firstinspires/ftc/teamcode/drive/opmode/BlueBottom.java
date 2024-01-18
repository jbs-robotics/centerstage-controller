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
@Autonomous(group = "drive", name="Blue Bottom", preselectTeleOp="Basic: Linear OpMode")
public class BlueBottom extends LinearOpMode {
    private DcMotor lift = null;
    private int liftDelay = 1000;
    private double intakeUp = 0.7, intakeDown = 0, clawUp = 0.5, clawDown = 0.4, angleServoUp = .1, angleServoDown = 0.43;

    private OpenCvCamera webcam = null;
    private ColorDetectorPipeline pipeline = null;
    private Servo intake = null, claw = null, fingerer;
    private CRServo angleServo = null;
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
        //home field: 40, 80 for x's
        pipeline.setRegionPoints(new Point(20, 140), new Point(60, 180), pipeline.getRegion2_pointA(), pipeline.getRegion2_pointB());
        telemetry.addData("region1_pointA: ", pipeline.getRegion1_pointA());
        telemetry.addData("region1_pointB: ", pipeline.getRegion1_pointB());
        telemetry.addData("region2_pointA: ", pipeline.getRegion2_pointA());
        telemetry.addData("region2_pointB: ", pipeline.getRegion2_pointB());
        telemetry.addData("PercentageLeft: ", pipeline.getPercentBlue1());
        telemetry.addData("PercentageRight: ", pipeline.getPercentBlue2());
        webcam.resumeViewport();

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

        telemetry.addData(">", "Press Play to start op mode");
        waitForStart();
        char TFODPrediction = pipeline.getAnalysis();
        drive.setPoseEstimate(new Pose2d(new Vector2d(-60, -37), Math.toRadians(0)));

        switch(TFODPrediction){
            case 'l': //left
                TrajectorySequence toSpikeLeft = drive.trajectorySequenceBuilder(new Pose2d(-60, -37, Math.toRadians(0)))
                        .splineToSplineHeading(new Pose2d(-34, -33, Math.toRadians(90)), Math.toRadians(90))
                        .forward(1)
//                        .back(12)
                        .build();
                drive.followTrajectorySequence(toSpikeLeft);

                //place pixel on spike mark
                placeOnSpike();

                TrajectorySequence toBackdropLeft = drive.trajectorySequenceBuilder(toSpikeLeft.end())
                        .back(6)
                        .lineToSplineHeading(new Pose2d(-3, -38, Math.toRadians(-90)))
                        .back(60)
                        .splineToConstantHeading(new Vector2d(-39, 45), Math.toRadians(180))
                        .back(6)
                        .build();
                drive.followTrajectorySequence(toBackdropLeft);

                //place pixel on canvas
                placeOnCanvas();

                break;
            case 'c': //center
                Trajectory center1 = drive.trajectoryBuilder(new Pose2d(-60, -37, Math.toRadians(0)))
                        .forward(28)
                        .build();
                drive.followTrajectory(center1);
//                Trajectory center1_2 = drive.trajectoryBuilder(center1.end())
//                        .back(10)
//                        .build();
//                drive.followTrajectory(center1_2);
//                Trajectory center1_3 = drive.trajectoryBuilder(center1_2.end())
                Trajectory center1_3 = drive.trajectoryBuilder(center1.end())
                        .strafeLeft(3)
                        .build();
                drive.followTrajectory(center1_3);

                //place pixel on spike mark
                placeOnSpike();

                Trajectory center1_4 = drive.trajectoryBuilder(center1_3.end())
                        .back(10)
                        .build();
                drive.followTrajectory(center1_4);

                Trajectory center1_5 = drive.trajectoryBuilder(center1_4.end())
                        .strafeRight(22)
                        .build();
                drive.followTrajectory(center1_5);

                Trajectory center1_6 = drive.trajectoryBuilder(center1_5.end())
                        .forward(24)
                        .build();
                drive.followTrajectory(center1_6);

                Trajectory center2 = drive.trajectoryBuilder(center1_6.end())
                        .splineToLinearHeading(new Pose2d(-10, -30, Math.toRadians(-90)), Math.toRadians(90))
                        .build();
                drive.followTrajectory(center2);

                Trajectory center2_1 = drive.trajectoryBuilder(center2.end())
                        .back(5)
                        .build();
                drive.followTrajectory(center2_1);

                Trajectory center2_2 = drive.trajectoryBuilder(center2_1.end())
                        .splineToConstantHeading(new Vector2d(-40, 38), Math.toRadians(-180))
                        .build();
                drive.followTrajectory(center2_2);

                Trajectory center3 = drive.trajectoryBuilder(center2_2.end())
                        .back(17)
                        .build();
                drive.followTrajectory(center3);
                //place pixel on canvas
                placeOnCanvas();

                break;
            case 'r': //right
                TrajectorySequence toSpikeRight = drive.trajectorySequenceBuilder(new Pose2d(-60, -37, Math.toRadians(0)))
                        .lineToSplineHeading(new Pose2d(-25, -35, Math.toRadians(-90)))
                        .build();
                drive.followTrajectorySequence(toSpikeRight);
                //place pixel on spike mark
                placeOnSpike();
                TrajectorySequence toBackdropRight = drive.trajectorySequenceBuilder(toSpikeRight.end())
                        .back(2)
                        .strafeLeft(20)
                        .back(20)
                        .splineToConstantHeading(new Vector2d(-26, 38), Math.toRadians(-180))
                        .back(14)
                        .build();
                drive.followTrajectorySequence(toBackdropRight);
                //place pixel on canvas
                placeOnCanvas();
                break;
            default:
                telemetry.addData("wtf how", "no but actually how");
                break;
        }

//        sleep(2000);

//        drive.followTrajectory(
//                drive.trajectoryBuilder(traj.end(), true)
//                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
//                        .build()
//        );
    }
    private void placeOnCanvas(){
        angleServo.setPower(-.2);
        sleep(2000);
        angleServo.setPower(0);
        claw.setPosition(clawUp);
        sleep(1000);
    }
    private void placeOnSpike(){
        fingerer.setPosition(0);
        sleep(500);
    }
}
