//package com.example.pathvisualizer.meet3;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.noahbres.meepmeep.MeepMeep;
//import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
//import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
//import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.opmodes.auto.PoseHelper;
//
//
//import java.awt.Image;
//import java.io.File;
//import java.io.IOException;
//
//import javax.imageio.ImageIO;
//
//public class PathVisualizer
//{
//    private enum StartPosition {
//        RED_CLOSE,
//        RED_FAR,
//        BLUE_CLOSE,
//        BLUE_FAR
//    }
//    private enum Path {
//        PLACEMENT,
//        INSIDE,
//        OUTSIDE
//    }
//
//    private static StartPosition startPosition = StartPosition.RED_CLOSE;
//    private Path path;
//
////    private Datalog datalog; //TELEMETRY
//
//
//    // TRAJECTORIES
//    private static TrajectorySequence init; // INIT THEN GO TO BACKBOARD
//    private TrajectorySequence backboardToSpike; // BACKBOARD TO SPIKE
//    private TrajectorySequence backboardToWhite; //BACKBOARD TO STACK
//    private TrajectorySequence spikeToStack; //SPIKE TO STACK
//    private TrajectorySequence whiteToBackboard;// STACK BACK TO BACKBOARD
//    private TrajectorySequence backboardToPark; //PARK AFTER SCORING STACK
//    private TrajectorySequence spikeToBackboard; //SPIKE TO BACKBOARD
//
//    //POSITIONS & TANGENTS
//    private Pose2d initPose; //INITIAL POSITION
//    private static Pose2d spikePose; //SPIKE POSITION
//    private Pose2d wingTruss; //WING TRUSS POSITION
//    private Pose2d boardTruss; //BOARD TRUSS POSITION
//    private Pose2d aprilTagPose; // POSITION AFTER APRIL TAGS
//    private Pose2d aprilTagPose2;
//    private Pose2d backboardPose; // POSITION WHERE BACKBOARD SHOULD BE, STARTING POSITION TO STACK
//    private double backboardApriltagX; // X VALUE OF LOCATION RELATIVE TO BACKBOARD IN INCHES
//    private double secondBackboardApriltagX; // X VALUE OF LOCATION RELATIVE TO BACKBOARD IN INCHES
//    private double initTangent; // INITIAL TANGENT 60 degrees
//    private double purpleTangent; // TANGENT TO SPIKE
//    private double centerBackup = 0; //BACKUP FROM SPIKE //TODO: WHY DO WE NEED THIS??????
//    private Pose2d beforeStackPose; //before we go to the stack,
//    private Pose2d preWhitePose;
//    private Pose2d whitePixelStackPose;
//    private Pose2d postLowerWhitePose; //pose after
//    private double preLowerWhiteTangent;
//    public static void main(String[] args) {
//        MeepMeep meepMeep = new MeepMeep(700);
//
//        buildAuto();
//        if(startPosition == StartPosition.RED_CLOSE || startPosition == StartPosition.BLUE_CLOSE) {
//            init = drive.trajectorySequenceBuilder(initPose)
//                    .splineToLinearHeading(backboardPose, Math.toRadians(180.0))
//                    .addTemporalMarker(() -> {
//                        alignToApriltagBackboard(); //TODO add apriltag errors
//                        drive.setPoseEstimate(backboardPose);
//                        drive.followTrajectorySequenceAsync(backboardToSpike); })
//                    .build();
//            backboardToSpike = drive.trajectorySequenceBuilder(backboardPose)
//                    .splineToLinearHeading(spikePose, spikePose.getHeading())
//                    .addTemporalMarker(this::outtakePurple)
//                    .addTemporalMarker(() -> {
//                        if(path != Path.PLACEMENT)
//                            drive.followTrajectorySequenceAsync(backboardToWhite);
//                        else
//                            drive.followTrajectorySequenceAsync(backboardToPark);
//                    })
//                    .build();
//            //TODO PARK
//        }
//        else { //FAR AUTO
//            init = drive.trajectorySequenceBuilder(initPose)
//                    .splineToLinearHeading(spikePose, spikePose.getHeading())
//                    .build();
//            spikeToStack = drive.trajectorySequenceBuilder(backboardPose)
//                    .addTemporalMarker(this::outtakePurple)
//                    .addTemporalMarker(() -> {
//                        intake.setServoPosition(Intake.IntakePositions.FIVE);
//                    })
//                    .setTangent(Math.toRadians(240))
//                    .splineToLinearHeading(PoseHelper.apriltagStackRed, Math.toRadians(180.0))
//                    .addTemporalMarker(() -> {
//                        alignToApriltagStack();
//                        drive.followTrajectorySequenceAsync(spikeToBackboard);
//                    })
//                    .build();
//            spikeToBackboard = drive.trajectorySequenceBuilder(spikePose)
//                    .addTemporalMarker(this::intake)
//                    .forward(4)
//                    .waitSeconds(1)
//                    .addTemporalMarker(this::stopIntake)
//                    .splineToSplineHeading(wingTruss, Math.toRadians(180.0))
//                    .splineToSplineHeading(boardTruss, Math.toRadians(180.0))
//                    .addTemporalMarker(this::outtake)
//                    .splineToLinearHeading(backboardPose, Math.toRadians(180.0))
//                    .addTemporalMarker(() -> {
//                        alignToApriltagBackboard();
//                        drive.setPoseEstimate(backboardPose); //TODO add apriltag errors
//                        if(path != Path.PLACEMENT)
//                            drive.followTrajectorySequenceAsync(backboardToWhite);
//                        else
//                            drive.followTrajectorySequenceAsync(backboardToPark);
//                    })
//                    .build();
//            //TODO PARK
//        }
//
//
//        //Trajectory strafeRight = new Trajectory()
//        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                .setDimensions(17.5,14.5)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                //.setStartPose(new Pose2d(50, 50))
//                .setConstraints(36, 36, Math.toRadians(180), Math.toRadians(180), 9)
//                .followTrajectorySequence(drive ->
//                                drive.trajectorySequenceBuilder(new Pose2d(-34,63, Math.toRadians(-90))) //initial position
//
//                                        .setTangent(Math.toRadians(-120))
//                                        .splineToSplineHeading(new Pose2d(-45,24, Math.toRadians(0)), Math.toRadians(-60))
//
//
//                                        // TODO: SPIKE PURPLE
//
//                                        .setTangent(Math.toRadians(-120))
//                                        // TODO: BRING SLIDE UP
//                                        .splineToLinearHeading(new Pose2d(-58,11,Math.toRadians(180)), Math.toRadians(-150))
//                                        .waitSeconds(1.0)
//                                        // TODO: OUTTAKE YELLOW
//                                        .setTangent(Math.toRadians(80))
//                                        // TODO: BRING SLIDE DOWN, RAISE INTAKE
//                                        .splineToConstantHeading(new Vector2d(-47,58), Math.toRadians(0))
//                                        // TODO: INTAKE 2 WHITE BOIS AND TRANSFER TO BOX
//
////                                        .splineToSplineHeading(new Pose2d(-66,-11.8, Math.toRadians(180)), Math.toRadians(180))
////                                        .lineToLinearHeading(new Pose2d(28, -11.8, Math.toRadians(180)))
//                                        // TODO: SLIDE UP
//                                        .splineToConstantHeading(new Vector2d(50,37), Math.toRadians(-40))
//                                        // TODO: OUTTAKE 2 WHITE BOIS
//
//                                        // TODO: SLIDE DOWN, INTAKE CHANGE POSITION
//                                        .build()
//                );
//        Image img = null;
//        try { img = ImageIO.read(new File("./PathVisualizer/src/main/java/com/example/pathvisualizer/centerstage_bg.png")); }
//        catch (IOException e) {}
//
//        meepMeep.setBackground(img)
//                .setDarkMode(true)
//                .setBackgroundAlpha(0.95f)
//                .addEntity(myBot)
//                .start();
//    }
//
//    private static void buildAuto() {
//        switch(startPosition) {
//            case RED_CLOSE:
//                initPose = PoseHelper.initCloseRed;
//                switch(path) {
//                    case OUTSIDE:
//                        wingTruss = PoseHelper.wingTrussRed;
//                        boardTruss = PoseHelper.boardTrussRed;
//                    case INSIDE:
//
//                }
//                switch(purplePixelPath) {
//                    case LEFT:
//                        spikePose = PoseHelper.closeSpikeLeftRed;
//                        break;
//                    case CENTER:
//                        spikePose = PoseHelper.closeSpikeCenterRed;
//                        break;
//                    case RIGHT:
//                        spikePose = PoseHelper.closeSpikeRightRed;
//                        break;
//                }
//                break;
//            case RED_FAR:
//                initPose = PoseHelper.initFarRed;
//                switch(path) {
//                    case OUTSIDE:
//                        wingTruss = PoseHelper.wingTrussRed;
//                        boardTruss = PoseHelper.boardTrussRed;
//                    case INSIDE:
//
//                }
//                switch(purplePixelPath) {
//                    case LEFT:
//                        spikePose = PoseHelper.farSpikeLeftRed;
//                        break;
//                    case CENTER:
//                        spikePose = PoseHelper.farSpikeCenterRed;
//                        break;
//                    case RIGHT:
//                        spikePose = PoseHelper.farSpikeRightRed;
//                        break;
//                }
//                break;
//            case BLUE_CLOSE:
//                initPose = PoseHelper.initCloseBlue;
//                switch(path) {
//                    case OUTSIDE:
//                        wingTruss = PoseHelper.wingTrussBlue;
//                        boardTruss = PoseHelper.boardTrussBlue;
//                    case INSIDE:
//
//                }
//                switch(purplePixelPath) {
//                    case LEFT:
//                        spikePose = PoseHelper.closeSpikeLeftBlue;
//                        break;
//                    case CENTER:
//                        spikePose = PoseHelper.closeSpikeCenterBlue;
//                        break;
//                    case RIGHT:
//                        spikePose = PoseHelper.closeSpikeRightBlue;
//                        break;
//                }
//                break;
//            case BLUE_FAR:
//                initPose = PoseHelper.initFarBlue;
//                switch(path) {
//                    case OUTSIDE:
//                        wingTruss = PoseHelper.wingTrussBlue;
//                        boardTruss = PoseHelper.boardTrussBlue;
//                    case INSIDE:
//
//                }
//                switch(purplePixelPath) {
//                    case LEFT:
//                        spikePose = PoseHelper.farSpikeLeftBlue;
//                        break;
//                    case CENTER:
//                        spikePose = PoseHelper.farSpikeCenterBlue;
//                        break;
//                    case RIGHT:
//                        spikePose = PoseHelper.farSpikeRightBlue;
//                        break;
//                }
//                break;
//        }
//    }
//}
//
