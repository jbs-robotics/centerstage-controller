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
@Autonomous(group = "drive", name="Blue Bottom")
public class BlueBottom extends LinearOpMode {
    private DcMotor lift = null;
    private Servo intake = null;
    boolean USE_WEBCAM = true;
    TfodProcessor tfod;
    private int liftDelay = 1000;

    private VisionPortal visionPortal;
//    private Servo intakeServo = null;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        intake = hardwareMap.get(Servo.class, "intake");
//        Servo rightServo = hardwareMap.get(Servo.class, "rightServo");

        lift.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(Servo.Direction.FORWARD);
        //        rightServo.setDirection(Servo.Direction.FORWARD);

//        drive.setPoseEstimate(new Pose2d(60, 10, Math.toRadians(180)));
        initTfod();
        lift = hardwareMap.get(DcMotor.class, "lift");
//        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        waitForStart();
//        telemetryTfod();
        telemetry.update();
//        List<Recognition> currentRecognitions = tfod.getRecognitions();
//        String TFODPrediction = currentRecognitions.get(0).getLabel();
        String TFODPrediction = "c";
        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d(new Vector2d(60, 10), Math.toRadians(180)));
        switch(TFODPrediction){
            case "l": //left
                Trajectory left1 = drive.trajectoryBuilder(new Pose2d(-60, -37, Math.toRadians(180)))
                        .lineToSplineHeading(new Pose2d(-35, -37, Math.toRadians(90)))
                        .build();
                drive.followTrajectory(left1);
                //place prop on spike mark
                Trajectory left2 = drive.trajectoryBuilder(left1.end())
                        .splineToConstantHeading(new Vector2d(-60, -37), Math.toRadians(360))
                        .splineToConstantHeading(new Vector2d(-55, -12), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(-48, -12), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(-42, 50), Math.toRadians(90))
                        .build();
                drive.followTrajectory(left2);
                //place pixel on canvas
                //TODO: figure out how much time is needed to lift the lift all the way up
                break;
            case "c": //center
                Trajectory center1 = drive.trajectoryBuilder(new Pose2d(-60, -37, Math.toRadians(180)))
                        .forward(23)
                        .build();
                drive.followTrajectory(center1);
                //place pixel on spike mark
                placeOnSpike();
//                Trajectory center1 = drive.trajectoryBuilder(new Pose2d(-60, -37, Math.toRadians(180)))
//                        .forward(27)
//                        .build();
//                drive.followTrajectory(center1);
//                //place pixel on spike mark
//                Trajectory center2 = drive.trajectoryBuilder(center1.end())
//                        .splineToSplineHeading(new Pose2d(-60, -37, Math.toRadians(90)), Math.toRadians(360))
//                        .splineToConstantHeading(new Vector2d(-55, -12), Math.toRadians(0))
//                        .splineToConstantHeading(new Vector2d(-48, -12), Math.toRadians(0))
//                        .splineToConstantHeading(new Vector2d(-35.5, 50), Math.toRadians(90))
//                        .build();
//                drive.followTrajectory(center2);
                //place pixel on canvas
                break;
            case "r": //right
                Trajectory right1 = drive.trajectoryBuilder(new Pose2d(-60, -37, Math.toRadians(0)))
                        .lineToSplineHeading(new Pose2d(-35, -37, Math.toRadians(-90)))
                        .build();
                Trajectory right2 = drive.trajectoryBuilder(right1.end())
                        .splineToSplineHeading(new Pose2d(-60, -37, Math.toRadians(90)), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(-55, -12), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(-48, -12), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(-28, 50), Math.toRadians(90))
                        .build();
                drive.followTrajectory(right1);
                //place pixel on spike mark
                drive.followTrajectory(right2);
                //place pixel on canvas
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
    private void initTfod() {
        tfod = tfod.easyCreateWithDefaults();
        /* non-default options for tfod
        * TfodProcessor.Builder myTfodProcessorBuilder;
          TfodProcessor myTfodProcessor;
          // Create a new TFOD Processor Builder object.
          myTfodProcessorBuilder = new TfodProcessor.Builder();

          // Optional: set other custom features of the TFOD Processor (4 are shown here).
          myTfodProcessorBuilder.setMaxNumRecognitions(10);  // Max. number of recognitions the network will return
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
    private void placeOnSpike(){
        lift.setPower(1);
        sleep(liftDelay/5);
        lift.setPower(0);
        intake.setPosition(0);
        sleep(4000);
        intake.setPosition(1);
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
