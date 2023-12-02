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

import java.util.List;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive", name="Blue Top")
public class BlueTop extends LinearOpMode {
    private DcMotor lift = null;
    private int liftDelay = 1000;

    boolean USE_WEBCAM = true;
    TfodProcessor tfod;
    private VisionPortal visionPortal;
    private Servo intake, lock = null;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        intake = hardwareMap.get(Servo.class, "intake");
        lock = hardwareMap.get(Servo.class, "lock");
        intake.setDirection(Servo.Direction.FORWARD);
        lock.setDirection(Servo.Direction.FORWARD);
//        initTfod();
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
//        telemetryTfod();
//        telemetry.update();
//        List<Recognition> currentRecognitions = tfod.getRecognitions();
//        String TFODPrediction = currentRecognitions.get(0).getLabel();
        String TFODPrediction = "c";
        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d(new Vector2d(-60, 10), Math.toRadians(0)));

        switch(TFODPrediction) {
            case "r": //right
                Trajectory right1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(new Vector2d(-25, 5), Math.toRadians(80)))
                        .build();
                drive.followTrajectory(right1);
                Trajectory forwardOffset = drive.trajectoryBuilder(right1.end())
                        .forward(5)
                        .build();
                drive.followTrajectory(forwardOffset);
                //place prop on spike mark
                placeOnSpike();
                Trajectory right2 = drive.trajectoryBuilder(forwardOffset.end())
                        .lineToLinearHeading(new Pose2d(new Vector2d(-18, 50), Math.toRadians(100.5)))
                        .build();
                drive.followTrajectory(right2);
                Trajectory offset2 = drive.trajectoryBuilder(right2.end())
                        .forward(10)
                        .build();

                drive.turn(Math.toRadians(30));
                drive.followTrajectory(offset2);
                //place pixel on canvas
                placeOnCanvas();
                lift.setPower(1);
                sleep(liftDelay);
                lift.setPower(0);
                break;
            case "c": //center
                Trajectory center1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .forward(20)
                        .build();
                drive.followTrajectory(center1);
                //place prop on spike mark
                placeOnSpike();
                Trajectory center2 = drive.trajectoryBuilder(center1.end())
                        .lineToLinearHeading(new Pose2d(-35.5, 50, Math.toRadians(90)))
                        .build();
                drive.followTrajectory(center2);
                drive.turn(Math.toRadians(25));
                Trajectory center3 = drive.trajectoryBuilder(center2.end())
                        .forward(4)
                        .build();
                drive.followTrajectory(center3);
                Trajectory center4 = drive.trajectoryBuilder(center3.end())
                        .strafeLeft(12)
                        .build();
                drive.followTrajectory(center4);
//                Trajectory center5 = drive.trajectoryBuilder(center4.end()).forward(5).build();
//                drive.followTrajectory(center5);
                //place pixel on canvas
                placeOnCanvas();
                lift.setPower(1);
                sleep(liftDelay);
                lift.setPower(0);
                break;
            case "l": //left
                Trajectory left1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .strafeRight(27)
                        .build();
                Trajectory left2 = drive.trajectoryBuilder(left1.end())
                        .lineToSplineHeading(new Pose2d(-30, 32, Math.toRadians(-90)))
                        .build();
                Trajectory left3 = drive.trajectoryBuilder(left2.end())
                        .forward(5)
                        .build();
                Trajectory left4 = drive.trajectoryBuilder(left3.end())
                        .strafeLeft(10)
                        .build();
                drive.followTrajectory(left1);
                drive.followTrajectory(left2);
                drive.followTrajectory(left3);
                drive.followTrajectory(left4);
//                drive.turn(Math.toRadians(-10));
                //place prop on spike mark
                placeOnSpike();
                Trajectory left5 = drive.trajectoryBuilder(left4.end())
                        .lineToSplineHeading(new Pose2d(-42, 50, Math.toRadians(115)))
                        .build();
                drive.followTrajectory(left5);
                drive.turn(Math.toRadians(-40));
                drive.followTrajectory(drive.trajectoryBuilder(left5.end()).forward(10).build());
                //place pixel on canvas
                placeOnCanvas();
                lift.setPower(1);
                sleep(liftDelay);
                lift.setPower(0);
                break;
            default:
                telemetry.addData("wtf how", "no but actually how");
                break;
        }
    }
    private void placeOnSpike(){
        lift.setPower(1);
        sleep(liftDelay/5);
        lift.setPower(0);
        intake.setPosition(0);
        sleep(4000);
        intake.setPosition(1);
    }
    private void placeOnCanvas(){
        lift.setPower(1);
        sleep(liftDelay/4);
        lift.setPower(0);
        lock.setPosition(1);
        intake.setPosition(0);
    }
    private void initTfod() {
        tfod = tfod.easyCreateWithDefaults();
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
        if (USE_WEBCAM){
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, tfod);
        }
    }
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }
}
