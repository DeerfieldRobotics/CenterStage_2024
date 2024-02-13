package com.example.pathvisualizer;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class BlueFar
{
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        //Trajectory strafeRight = new Trajectory()
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(17.5,14.5)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                //.setStartPose(new Pose2d(50, 50))
                .setConstraints(36, 36, Math.toRadians(180), Math.toRadians(180), 9)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-34,63, Math.toRadians(-90))) //initial position

                                        .setTangent(Math.toRadians(-120))
                                        .splineToSplineHeading(new Pose2d(-45,24, Math.toRadians(0)), Math.toRadians(-60))


                                        // TODO: SPIKE PURPLE

                                        .setTangent(Math.toRadians(-120))
                                        // TODO: BRING SLIDE UP
                                        .splineToLinearHeading(new Pose2d(-58,11,Math.toRadians(180)), Math.toRadians(-150))
                                        .waitSeconds(1.0)
                                        // TODO: OUTTAKE YELLOW
                                        .setTangent(Math.toRadians(80))
                                        // TODO: BRING SLIDE DOWN, RAISE INTAKE
                                        .splineToConstantHeading(new Vector2d(-47,58), Math.toRadians(0))
                                        // TODO: INTAKE 2 WHITE BOIS AND TRANSFER TO BOX

//                                        .splineToSplineHeading(new Pose2d(-66,-11.8, Math.toRadians(180)), Math.toRadians(180))
//                                        .lineToLinearHeading(new Pose2d(28, -11.8, Math.toRadians(180)))
                                        // TODO: SLIDE UP
                                        .splineToConstantHeading(new Vector2d(50,37), Math.toRadians(-40))
                                        // TODO: OUTTAKE 2 WHITE BOIS

                                        // TODO: SLIDE DOWN, INTAKE CHANGE POSITION
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