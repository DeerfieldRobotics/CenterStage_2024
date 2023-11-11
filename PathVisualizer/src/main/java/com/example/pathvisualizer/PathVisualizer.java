package com.example.pathvisualizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class PathVisualizer {
    public static void main(String[] args) {
        double mult = -0.8;
        MeepMeep meepMeep = new MeepMeep(700);

        //Trajectory strafeRight = new Trajectory()
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14.25,17.75).setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 13.5)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(5,-63, Math.toRadians(90)))
                                        .splineToSplineHeading(new Pose2d(10-mult*3,-34+(1-Math.abs(mult)), Math.toRadians(90+52*mult)), Math.toRadians(90+52*mult)) //drop off purple
                                        //TODO: OUTTAKE PURPLE HERE
                                        .back(5)
                                        .splineToLinearHeading(new Pose2d(19, -48, Math.toRadians(180)), Math.toRadians(0))
                                        .setTangent(Math.toRadians(0))
                                        //.strafeRight(14)
                                        .lineToLinearHeading(new Pose2d(50,-35+mult*7,Math.toRadians(180)))
                                        .splineToSplineHeading(new Pose2d(50,-35+mult*7,Math.toRadians(180)), Math.toRadians(0))
                                        .waitSeconds(2.0).build()


                );
        Image img = null;
        try { img = ImageIO.read(new File("./PathVisualizer/src/main/java/com/example/pathvisualizer/centerstage_bg.png")); }
        catch (IOException e) {}

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}