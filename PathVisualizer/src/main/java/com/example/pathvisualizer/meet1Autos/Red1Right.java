package com.example.pathvisualizer.meet1Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class Red1Right {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        //Trajectory strafeRight = new Trajectory()
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(15,14)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                //.setStartPose(new Pose2d(50, 50))
                .setConstraints(36, 36, Math.toRadians(180), Math.toRadians(180), 13.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11,-63, Math.toRadians(90))) //initial position
                                .strafeRight(2)
                                .waitSeconds((0.05))
                                .splineToSplineHeading(new Pose2d(10,-34,Math.toRadians(90+45*-1)), Math.toRadians(90+45*-1)) //drop off purple
                                .back(2)
                                .setTangent(Math.toRadians(-45))
                                .splineToLinearHeading(new Pose2d(50,-35,Math.toRadians(180)), Math.toRadians(45))
                                .waitSeconds(2.0)
                                //TODO: OUTTAKE YELLOW HERE, BRING SLIDE UP AND OUTTAKE

                                .setTangent(135)
                                .splineToConstantHeading(new Vector2d(24,-11.8), Math.toRadians(180))

                                .splineToConstantHeading(new Vector2d(-66,-11.8), Math.toRadians(180))

                                .waitSeconds(1.0)
                                .back(10)
                                .waitSeconds(1.0)
                                .lineToLinearHeading(new Pose2d(28, -11.8, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(50,-37), Math.toRadians(-45))


//                                //TODO: OUTTAKE PURPLE HERE
//
//                                .setTangent(Math.toRadians(270))
//                                .splineToSplineHeading(new Pose2d(-57,-55, Math.toRadians(180)), Math.toRadians(180))
//                                .lineToLinearHeading(new Pose2d(-57,-11.8, Math.toRadians(180)))
//                                .lineToLinearHeading(new Pose2d(-60, -11.8, Math.toRadians(180)))
//                                .waitSeconds(2)
//                                .lineToLinearHeading(new Pose2d(35, -11.8, Math.toRadians(180)))
//                                .lineToLinearHeading(new Pose2d(52, -35,Math.toRadians(180)))
//                                .waitSeconds(2)
//                                .lineToLinearHeading(new Pose2d(35, -11.8, Math.toRadians(180)))
//                                .lineToLinearHeading(new Pose2d(-60, -11.8, Math.toRadians(180)))
//                                .waitSeconds(2)
//                                .lineToLinearHeading(new Pose2d(35, -11.8, Math.toRadians(180)))
//                                .lineToLinearHeading(new Pose2d(52, -35,Math.toRadians(180)))
//                                .waitSeconds(2)
//                                .forward(2)
//                                .lineToLinearHeading(new Pose2d(52, -11.8, Math.toRadians(180)))
//                                .lineToLinearHeading(new Pose2d(60, -11.8, Math.toRadians(180)))

//                                .splineToLinearHeading(new Pose2d(10,-40, Math.toRadians(135)), Math.toRadians(135)) //drop off purple
//                                //TODO: OUTTAKE PURPLE HERE
//                                .setTangent(Math.toRadians(0))
//                                .lineToLinearHeading(new Pose2d(50,-35,Math.toRadians(180))) //drop off yellow
//                                //TODO: OUTTAKE YELLOW HERE, BRING SLIDE UP AND OUTTAKE
//                                /*
//                                .back(20)
//                                .turn(Math.toRadians(180))
//                                 */
////                                .lineToLinearHeading(new Pose2d(-30, -35,Math.toRadians(0)))
//                                .lineToLinearHeading(new Pose2d(-60, -35,Math.toRadians(180))) //Go back to stack
//                                //TODO: ADD INTAKE HERE
//                                .lineToLinearHeading(new Pose2d(50, -35,Math.toRadians(180))) //drop off pixel
//                                //TODO: OUTTAKE PIXEL HERE
//                                .forward(5)
//                                .strafeRight(10)
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