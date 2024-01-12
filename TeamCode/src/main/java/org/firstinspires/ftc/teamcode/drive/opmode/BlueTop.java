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

    private OpenCvCamera webcam = null;
    private ColorDetectorPipeline pipeline = null;
    private Servo intake = null, angleServo = null, claw = null;
    private double intakeUp = 0.7, intakeDown = 0, clawUp = 0.5, clawDown = 0.4, angleServoUp = .5, angleServoDown = 0.43;
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
        intake.setDirection(Servo.Direction.FORWARD);
        angleServo = hardwareMap.get(Servo.class, "angleServo");
        claw = hardwareMap.get(Servo.class, "claw");
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setDirection(DcMotor.Direction.REVERSE);
        intake.setPosition(intakeUp);
        claw.setPosition(clawDown);

        telemetry.addData(">", "Press Play to start op mode");
        waitForStart();
        char TFODPrediction = pipeline.getAnalysis();
        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d(new Vector2d(-60, 10), Math.toRadians(0)));

        switch(TFODPrediction) {
            case 'r': //right
                Trajectory right1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(new Vector2d(-26, 14), Math.toRadians(-90)))
                        .build();
                drive.followTrajectory(right1);
                Trajectory right1_2_1 = drive.trajectoryBuilder(right1.end())
                        .strafeRight(3)
                        .build();
                Trajectory right1_2 = drive.trajectoryBuilder(right1_2_1.end())
                        .forward(12)
                        .build();
                Trajectory right1_3 = drive.trajectoryBuilder(right1_2.end())
                        .back(12)
                        .build();
                drive.followTrajectory(right1_2);
                drive.followTrajectory(right1_3);
                //place prop on spike mark
                placeOnSpike();

                Trajectory right2 = drive.trajectoryBuilder(right1.end())
                        .lineToLinearHeading(new Pose2d(new Vector2d(-18, 50), Math.toRadians(90)))
                        .build();
                drive.followTrajectory(right2);
                Trajectory right3 = drive.trajectoryBuilder(right2.end().plus(new Pose2d(0, 0, Math.toRadians(180))))
                        .lineToConstantHeading(new Vector2d(-26, 54))
                        .build();
                drive.turn(Math.toRadians(180));
                drive.followTrajectory(right3);
                //place pixel on canvas
                placeOnCanvas();
                break;
            case 'c': //center
                Trajectory center1 = drive.trajectoryBuilder(drive.getPoseEstimate())
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
                        .lineToLinearHeading(new Pose2d(-35.5, 50, Math.toRadians(90)))
                        .build();
                drive.followTrajectory(center1_5);
                drive.followTrajectory(center2);
                Trajectory center3 = drive.trajectoryBuilder(center2.end().plus(new Pose2d(0, 0, Math.toRadians(180))))
                        .lineToConstantHeading(new Vector2d(-36.5, 53.5))
                        .build();
                drive.turn(Math.toRadians(180));
                drive.followTrajectory(center3);
                //place pixel on canvas
                placeOnCanvas();
                break;
            case 'l': //left
                Trajectory left1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .strafeLeft(27)
                        .build();
                Trajectory left2 = drive.trajectoryBuilder(left1.end())
                        .lineToSplineHeading(new Pose2d(-30, 39, Math.toRadians(-90)))
                        .build();
                drive.followTrajectory(left1);
                drive.followTrajectory(left2);
                //place prop on spike mark
                placeOnSpike();
                Trajectory left3 = drive.trajectoryBuilder(left2.end())
                        .lineToSplineHeading(new Pose2d(-42, 50, Math.toRadians(115)))
                        .build();
                drive.followTrajectory(left3);
                Trajectory left4 = drive.trajectoryBuilder(left3.end().plus(new Pose2d(0, 0, Math.toRadians(180))))
                        .lineToConstantHeading(new Vector2d(-42, 54))
                        .build();
                drive.turn(Math.toRadians(180));
                drive.followTrajectory(left4);
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
        intake.setPosition(.3);
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
