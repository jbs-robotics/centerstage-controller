package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive", name="TEST")
public class RedTopTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int TFODPrediction = 0;
        waitForStart();

        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d(new Vector2d(60, 10), Math.toRadians(180)));
        switch(TFODPrediction){
            case 0: //left
                Trajectory left1 = drive.trajectoryBuilder(new Pose2d(60, 10, Math.toRadians(180)))
                        .lineToLinearHeading(new Pose2d(new Vector2d(35, 10), Math.toRadians(-80)))
                        .build();
                drive.followTrajectory(left1);
                //place prop on spike mark
                Trajectory left2 = drive.trajectoryBuilder(left1.end())
                        .lineToLinearHeading(new Pose2d(new Vector2d(28, 50), Math.toRadians(180)))
                        .build();
                drive.followTrajectory(left2);
                //place pixel on canvas
                break;
            case 1: //center
                Trajectory center1 = drive.trajectoryBuilder(new Pose2d(60, 10, Math.toRadians(180)))
                        .forward(27)
                        .build();
                drive.followTrajectory(center1);
                //place prop on spike mark
                Trajectory center2 = drive.trajectoryBuilder(center1.end())
                        .lineToLinearHeading(new Pose2d(35.5, 50, Math.toRadians(90)))
                        .build();
                drive.followTrajectory(center2);
                //place pixel on canvas
                break;
            case 2: //right
                Trajectory right1 = drive.trajectoryBuilder(new Pose2d(60, 10, Math.toRadians(180)))
                        .strafeRight(27)
                        .build();
                Trajectory right2 = drive.trajectoryBuilder(right1.end())
                        .lineToSplineHeading(new Pose2d(30, 32, Math.toRadians(-90)))
                        .build();
                drive.followTrajectory(right1);
                drive.followTrajectory(right2);
                //place prop on spike mark
                Trajectory right3 = drive.trajectoryBuilder(right2.end())
                        .lineToSplineHeading(new Pose2d(42, 50, Math.toRadians(90)))
                        .build();
                drive.followTrajectory(right3);
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
}
