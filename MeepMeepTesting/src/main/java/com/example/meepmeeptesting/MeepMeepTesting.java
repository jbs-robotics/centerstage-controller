package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;
import java.util.Vector;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        //Red Top
//        RoadRunnerBotEntity RTL = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
//                //left
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(60, 10, Math.toRadians(180)))
//                                .lineToSplineHeading(new Pose2d(35, 10, Math.toRadians(-90)))
//                                //place pixel
//                                .lineToSplineHeading(new Pose2d(28, 50, Math.toRadians(90)))
//                                //place other pixel on backdrop
//                                .build()
//                );
//        RoadRunnerBotEntity RTC = new DefaultBotBuilder(meepMeep)
//                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
//                //center
//                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(60, 10, Math.toRadians(180)))
//                        .forward(27)
//                        //place pixel
//                        .lineToSplineHeading(new Pose2d(35.5, 50, Math.toRadians(90)))
//                        //place pixel on canvas
//                        .build()
//                );
//        RoadRunnerBotEntity RTR = new DefaultBotBuilder(meepMeep)
//                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
//                //right
//                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(60, 10, Math.toRadians(180)))
//                        .strafeRight(25)
//                        .lineToSplineHeading(new Pose2d(30, 32, Math.toRadians(-90)))
//                        //place pixel
//                        .lineToSplineHeading(new Pose2d(42, 50, Math.toRadians(90)))
//                        .build()
//                );
        //Blue Top
        RoadRunnerBotEntity BTR = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                //left
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-60, 10, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(-35, 10, Math.toRadians(-90)))
                                //place pixel
                                .lineToSplineHeading(new Pose2d(-28, 50, Math.toRadians(90)))
                                //place other pixel on backdrop
                                .build()
                );
        RoadRunnerBotEntity BTC = new DefaultBotBuilder(meepMeep)
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                //center
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-60, 10, Math.toRadians(0)))
                        .forward(27)
                        //place pixel
                        .lineToSplineHeading(new Pose2d(-35.5, 50, Math.toRadians(90)))
                        //place pixel on canvas
                        .build()
                );
        RoadRunnerBotEntity BTL = new DefaultBotBuilder(meepMeep)
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                //right
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-60, 10, Math.toRadians(0)))
                        .strafeLeft(25)
                        .lineToSplineHeading(new Pose2d(-30, 32, Math.toRadians(-90)))
                        //place pixel
                        .lineToSplineHeading(new Pose2d(-42, 50, Math.toRadians(90)))
                        .build()
                );
        //Blue Bottom
        RoadRunnerBotEntity BBR = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                //right
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-60, -37, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(-35, -37, Math.toRadians(-90)))
                                //place pixel
                                .splineToSplineHeading(new Pose2d(-60, -37, Math.toRadians(90)), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(-55, -12), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(-48, -12), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(-28, 50), Math.toRadians(90))
                                //place pixel on canvas
                                .build()
                );
        RoadRunnerBotEntity BBC = new DefaultBotBuilder(meepMeep)
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                //center
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-60, -37, Math.toRadians(0)))
                        .forward(27)
                        //place pixel
                        .splineToSplineHeading(new Pose2d(-60, -37, Math.toRadians(90)), Math.toRadians(360))
                        .splineToConstantHeading(new Vector2d(-55, -12), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(-48, -12), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(-35.5, 50), Math.toRadians(90))
                        //place pixel on canvas
                        .build()
                );
        RoadRunnerBotEntity BBL = new DefaultBotBuilder(meepMeep)
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                //left
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-60, -37, Math.toRadians(0)))
                        .lineToSplineHeading(new Pose2d(-35, -37, Math.toRadians(90)))
                        //place pixel
                        .splineToConstantHeading(new Vector2d(-60, -37), Math.toRadians(360))
                        .splineToConstantHeading(new Vector2d(-55, -12), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(-48, -12), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(-42, 50), Math.toRadians(90))
                        //place pixel on canvas
                        .build()
                );
        Image img = null;
        System.out.print(System.getProperty("user.dir") + '\n');
        try {img = ImageIO.read(new File(System.getProperty("user.dir") + "/centerstage_background.png"));
            meepMeep.setBackground(img);
        }
        catch(IOException e) {System.out.println(e);}
//        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
        meepMeep.setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(BBC)
                .addEntity(BBL)
                .addEntity(BBR)
                .start();
    }
}