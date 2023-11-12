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
        double righty = 0.0;
        double lefty = 0.0;
        double centery = 0.0;
        double rightBackboard = 0.0;
        double centerBackboard = 0.0;
        double leftConst = 0.0;
        double centerx = 5;

        //Trajectory strafeRight = new Trajectory()
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14.25,17.75).setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 13.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(16.5,63, Math.toRadians(270)))
                                .strafeRight(5)
                                .waitSeconds(0.05)
                                .setTangent(-45)
                                .splineToSplineHeading(new Pose2d(11.5-4.5*mult+centerx,36-(4-4*Math.abs(mult)),Math.toRadians(270+53*mult)), Math.toRadians(270+53*mult)) //drop off purple
                                .back(3)
                                .setTangent(Math.toRadians(45))
                                .splineToLinearHeading(new Pose2d(55+leftConst,33+rightBackboard+centerBackboard+7*mult,Math.toRadians(180)), Math.toRadians(-45))
                                .waitSeconds(0.3)
                                .waitSeconds(0.4)
                                .waitSeconds(0.5)
                                .waitSeconds(0.4)
                                //TODO: OUTTAKE YELLOW HERE, BRING SLIDE UP AND OUTTAKE

                                .setTangent(135)
                                .splineToConstantHeading(new Vector2d(24,10.5), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-61,11.6-0.4*mult+righty+lefty+centery), Math.toRadians(180))
                                //INTAKE
                                .waitSeconds(0.8)
                                .back(6)
                                .waitSeconds(0.5)
                                .waitSeconds(1.5)
                                .waitSeconds(0.2)
                                .lineToLinearHeading(new Pose2d(28, 10.5, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(55,36), Math.toRadians(45))
                                //OUTTAKE 2
                                .waitSeconds(0.4)
                                .waitSeconds(0.2)
                                .waitSeconds(0.4)
                                .waitSeconds(0.4)
                                //PARK
                                .forward(5)
                                .strafeLeft(27)
                                .back(10)
                                .waitSeconds(10)
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