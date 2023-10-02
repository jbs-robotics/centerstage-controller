package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity leftBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(60, 10, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(35, 10, Math.toRadians(-90)))
                                //place pixel
                                .lineToSplineHeading(new Pose2d(28, 50, Math.toRadians(90)))
                                //place other pixel on backdrop
                                .build()
                );
        RoadRunnerBotEntity centerBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(60, 10, Math.toRadians(180)))
                        .forward(27)
                        //place pixel
                        .lineToSplineHeading(new Pose2d(35.5, 50, Math.toRadians(90)))
                        //place pixel on canvas
                        .build()
                );
        RoadRunnerBotEntity rightBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(60, 10, Math.toRadians(180)))
                        .strafeRight(25)
                        .lineToSplineHeading(new Pose2d(30, 32, Math.toRadians(-90)))
                        //place pixel
                        .lineToSplineHeading(new Pose2d(42, 50, Math.toRadians(90)))
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
                .addEntity(centerBot)
                .addEntity(leftBot)
                .addEntity(rightBot)
                .start();
    }
}