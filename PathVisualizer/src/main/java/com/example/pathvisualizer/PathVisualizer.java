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
    static private Pose2d initPose;
    static private Pose2d purplePose;
    static private Pose2d aprilTagPose;
    static private Pose2d backboardPose;
    static private double initTangent;
    static private double purpleTangent;
    static private double centerBackup = 0;
    static private Pose2d whiteDetectionPose;
    static private Pose2d preWhitePose;
    static private Pose2d whitePixelStackPose;
    static private Pose2d postLowerWhitePose;
    static private double preLowerWhiteTangent;
    static private Pose2d firstWhitePickup;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        PurplePixelPath purplePixelPath = PurplePixelPath.CENTER;

        initPose = new Pose2d(11, -63, Math.toRadians(90));
        purplePose = new Pose2d(-63, -48, Math.toRadians(0));
        purpleTangent = Math.toRadians(150);
        initTangent = Math.toRadians(90);
        whiteDetectionPose = new Pose2d(24,-10, Math.toRadians(180));
        whitePixelStackPose = new Pose2d(-57,16, Math.toRadians(180));
        preWhitePose = new Pose2d(24, -10, Math.toRadians(180));
        postLowerWhitePose = new Pose2d(28, -10, Math.toRadians(180));
        preLowerWhiteTangent = 135;
        switch (purplePixelPath) {
            case LEFT:
            purplePose = new Pose2d(11,-32, Math.toRadians(180));
            aprilTagPose = new Pose2d(-63, -48, Math.toRadians(0)); // TODO see below
            backboardPose = new Pose2d(-65, -48, Math.toRadians(0)); // TODO see below
            centerBackup = 1;
            break;
            case CENTER:
                purplePose = new Pose2d(23,-24.2, Math.toRadians(180));
                aprilTagPose = new Pose2d(-63, -48, Math.toRadians(0)); // TODO see below
                backboardPose = new Pose2d(-65, -48, Math.toRadians(0)); // TODO see below
                centerBackup = 1;
                break;
            case RIGHT:
                purplePose = new Pose2d(25,-32, Math.toRadians(180));
                aprilTagPose = new Pose2d(-63, -48, Math.toRadians(0)); // TODO see below
                backboardPose = new Pose2d(-65, -48, Math.toRadians(0)); // TODO see below
                centerBackup = 1;
                break;
        }

        //Trajectory strafeRight = new Trajectory()
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(14.25,17.75).setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 13.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(initPose)
                                .setTangent(initTangent)
                                //.addTemporalMarker(()->{ intake.setServoPosition(Intake.IntakePositions.DRIVE); })
                                .splineToSplineHeading(purplePose, purpleTangent)
                                .back(centerBackup)
                                //.addTemporalMarker(this::outtakePurple)
                                .waitSeconds(0.2)
                                .setTangent(0)
                                .splineToLinearHeading(aprilTagPose, Math.toRadians(0))

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

enum PurplePixelPath {
    LEFT,
    CENTER,
    RIGHT
}