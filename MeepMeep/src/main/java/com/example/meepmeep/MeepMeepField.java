package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepField {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(26, 63, Math.toRadians(0)))
                        //initial placement
                        .splineTo(new Vector2d(58, 63), Math.toRadians(40))

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}

//VERSION 1 LADDER AUTO
/*
//initial placement
                        .splineToConstantHeading(new Vector2d(0, 35), Math.toRadians(-90))
                        .forward(5)
                        .back(5)

                        //connection
                        .splineToConstantHeading(new Vector2d(-36, 30), Math.toRadians(-90))
                        .forward(10)

                        //cycles in terminal
                        .splineToConstantHeading(new Vector2d(-47, 10), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-47, 55), Math.toRadians(-90))

                        .splineToConstantHeading(new Vector2d(-58, 10), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-58, 55), Math.toRadians(-90))

                        .splineToConstantHeading(new Vector2d(-65, 10), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-65, 55), Math.toRadians(-90))

                        .splineToConstantHeading(new Vector2d(-55, 35), Math.toRadians(-90))
                        // claw out
                        .back(10)
                        .forward(10)
                        //claw in

                        //cycle 1
                        .splineToConstantHeading(new Vector2d(0, 35), Math.toRadians(-90))
                        .waitSeconds(2)
                        .lineTo(new Vector2d(-55, 40))
                        .back(10)
                        .forward(10)

                        //cycle 2
                        .splineToConstantHeading(new Vector2d(0, 35), Math.toRadians(-90))
                        .waitSeconds(2)
                        .lineTo(new Vector2d(-55, 40))
                        .back(10)
                        .forward(10)

                        //cycle 3
                        .splineToConstantHeading(new Vector2d(0, 35), Math.toRadians(-90))
                        .forward(3)
                        .waitSeconds(2)

                        .build());
 */