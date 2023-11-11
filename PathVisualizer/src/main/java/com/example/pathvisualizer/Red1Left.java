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

public class Red1Left {
    private static double mult = 0.0;
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        //Trajectory strafeRight = new Trajectory()
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(15,14)

                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                //.setStartPose(new Pose2d(50, 50))
                .setConstraints(36, 36, Math.toRadians(180), Math.toRadians(180), 13.5)
                .followTrajectorySequence(drive ->
                    drive.trajectorySequenceBuilder(new Pose2d(-39.75,-63, Math.toRadians(90)))
        //                .addTemporalMarker(3.5, ()->{
        //                    slide.setTargetPosition(-990);
        //                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //                    slide.setPower(-1);
        //                })
                        .strafeRight(5)
                        .waitSeconds(0.05)
                        .splineToSplineHeading(new Pose2d(-36.5-4.5*mult,-34+(1-1*Math.abs(mult)),Math.toRadians(90+53*mult)), Math.toRadians(90+53*mult)) //drop off purple
        //                .addTemporalMarker(()->{
        //                    intake.getIntakeServo().setPosition(0.9);
        //                })
                        .back(2)
                        .setTangent(Math.toRadians(270))
                            .splineToSplineHeading(new Pose2d(-44.5, -45.5, Math.toRadians(180)), Math.toRadians(225))
                            .splineToLinearHeading(new Pose2d(-57,-15, Math.toRadians(180)), Math.toRadians(90))
                            .splineToLinearHeading(new Pose2d(-61,-12, Math.toRadians(180)), Math.toRadians(180))
                            //INTAKE
                            //                .addTemporalMarker(()->{
                            //                    slide.setPower(0);
                            //                    slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            //                    intake.intake(0.35);
                            //                    intake.getIntakeServo().setPosition(0.8);
                            //                })
                            .waitSeconds(0.6)
                            //                .addTemporalMarker(()->{
                            //                    intake.intake(0.0);
                            //                })
                            .back(9)
                            //                .addTemporalMarker(()->{
                            //                    intake.getIntakeServo().setPosition(0.0);
                            //                })
                            .waitSeconds(0.5)
                            //                .addTemporalMarker(()->{
                            //                    intake.intake(0.7);
                            //                })
                            .waitSeconds(1.3)
                            //                .addTemporalMarker(()->{
                            //                    intake.intake(0.0);
                            //                    intake.outtakeToggle();
                            //                    intake.getIntakeServo().setPosition(1.0);
                            //                })
                            .lineToLinearHeading(new Pose2d(28, -12, Math.toRadians(180)))
                            //                .addTemporalMarker(()->{
                            //                    slide.setTargetPosition(-1400);
                            //                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            //                    slide.setPower(-1);
                            //                })
                            .splineToConstantHeading(new Vector2d(54,-33), Math.toRadians(-45))
                            //OUTTAKE 2
                            //                .addTemporalMarker(()->{
                            //                    intake.armToggle();
                            //                })
                            .waitSeconds(0.4)
                            //                .addTemporalMarker(()->{
                            //                    intake.getOuttakeServo().setPosition(0.34);
                            //                })
                            .waitSeconds(0.2)
                            //                .addTemporalMarker(()->{
                            //                    slide.setTargetPosition(-1300);
                            //                })
                            .waitSeconds(0.4)
                            //                .addTemporalMarker(()->{
                            //                    intake.armToggle();
                            //                })
                            .waitSeconds(0.4)
                            //                .addTemporalMarker(()->{
                            //                    slide.setTargetPosition(0);
                            //                    slide.setPower(1);
                            //                })
                            //PARK
                            .forward(5)
                            .strafeRight(27)
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