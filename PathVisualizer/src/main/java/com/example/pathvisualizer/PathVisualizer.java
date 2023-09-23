package com.example.pathvisualizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class PathVisualizer {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        //Trajectory strafeRight = new Trajectory()
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(15,14)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                //.setStartPose(new Pose2d(50, 50))
                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 13.5)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-36,-63, Math.toRadians(0)))
                                        .strafeLeft(2)
                                        .setTangent(0)
//                            .lineToSplineHeading(new Pose2d(-62,-12,Math.toRadians(180)))
//
//                            .splineToSplineHeading(new Pose2d(-28.5,-7.5,Math.toRadians(80)),Math.toRadians(30))

                                        .splineTo(new Vector2d(-17, -48), Math.toRadians(50))
                                        .splineToSplineHeading(new Pose2d(-8, -15, Math.toRadians(-40)), Math.toRadians(0))
                                        //.setTangent(90)
                                        //.splineToSplineHeading(new Pose2d(-7, -24, Math.toRadians(-30)), Math.toRadians(-30))



                                        .build()


                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}