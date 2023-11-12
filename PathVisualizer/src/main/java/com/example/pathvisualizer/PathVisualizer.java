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
        MeepMeep meepMeep = new MeepMeep(700);

        double mult = 0.0;
        int centerx = 5;

        //Trajectory strafeRight = new Trajectory()
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14.25,17.75).setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 13.5)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(8.25,63, Math.toRadians(270)))

                                        .strafeLeft(5)
                                        .waitSeconds(0.05)
                                        .splineToSplineHeading(new Pose2d(11.5-4.5*mult+centerx,36-(4-4*Math.abs(mult)),Math.toRadians(270-53*mult)), Math.toRadians(270-53*mult)) //drop off purple

                                        .back(2)
                                        .setTangent(Math.toRadians(45))
                                        .splineToLinearHeading(new Pose2d(55,35-6.5*mult,Math.toRadians(0)), Math.toRadians(-45))
                                        .waitSeconds(0.3)
                                        .waitSeconds(0.4)
                                        .waitSeconds(0.5)
                                        .waitSeconds(0.4)
                                        .setTangent(135)
                                        .splineToConstantHeading(new Vector2d(24,10.5), Math.toRadians(180))
                                        .splineToConstantHeading(new Vector2d(-61,12+0.4*mult), Math.toRadians(180))
                                        //INTAKE
                        //TODO: OUTTAKE YELLOW HERE, BRING SLIDE UP AND OUTTAKE
                                        .build()


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