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

public class Blue1Left {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        //Trajectory strafeRight = new Trajectory()
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(15,14)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                //.setStartPose(new Pose2d(50, 50))
                .setConstraints(36, 36, Math.toRadians(180), Math.toRadians(180), 13.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-38,63, Math.toRadians(-90))) //initial position
                                .splineToSplineHeading(new Pose2d(-35,34,Math.toRadians(-90)), Math.toRadians(-90)) //drop off purple
                                .setTangent(Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(-50,50,Math.toRadians(180)), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-66,11.8), Math.toRadians(180))

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