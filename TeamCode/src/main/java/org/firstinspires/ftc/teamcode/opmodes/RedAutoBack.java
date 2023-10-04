//package org.firstinspires.ftc.teamcode.opmodes;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.utils.AprilTags;
//import org.firstinspires.ftc.teamcode.utils.Tag;
//
//@Autonomous(name = "CONE_AUTO")
//public class coneAuto extends OpMode {
//    private SampleMecanumDrive drive;
//    private AprilTags aprilTags;
//    private Tag[] detectedTags;
//
//    TrajectorySequence path;
//    Pose2d startPose = new Pose2d(36, -63, Math.toRadians(180));
//
//    @Override
//    public void init() {
//        drive = new SampleMecanumDrive(hardwareMap);
//        aprilTags = new AprilTags(hardwareMap, "Webcam 1");
//
//
//    }
//
//    public void init_loop() {
//        drive.setPoseEstimate(startPose);
//        detectedTags = aprilTags.getTags();
//        for(Tag t : detectedTags) telemetry.addLine(t.toString());
//        telemetry.update();
//    }
//
//    public void loop() {
//        drive.update();
//        detectedTags = aprilTags.getTags();
//        for(Tag t : detectedTags) telemetry.addLine(t.toString());
//        telemetry.update();
////        s.update();
////        rubberBandIntake.update();
//    }
//
//    public void start() {
////        if (detectedTag == 3) {
////            path = drive.trajectorySequenceBuilder(startPose)
//                    .strafeRight(3)
//                    .forward(3)
//                    .splineTo(new Vector2d(14,-50), Math.toRadians(115))
//                    .splineTo(new Vector2d(5,-33), Math.toRadians(142))
//                    .addTemporalMarker(3, () -> {
////                        s.setTarget(-3400);
//                    })
//
//                    .forward(2)
//                    .addTemporalMarker(4.25,() ->{
//                        telemetry.update();
////                        rubberBandIntake.updatePower(-1);
//                    })
//                    .addTemporalMarker(4.75, ()-> {
//                        telemetry.update();
////                        rubberBandIntake.updatePower(0);
//                    })
//                    .waitSeconds(0.5)
//                    .back(10)
//                    .turn(Math.toRadians(-53))
//                    .addDisplacementMarker(() ->{
////                        s.setTarget(0);
//                    })
//                    .strafeRight(27)
//                    .build();
//        } else if (detectedTag == 2) {
//            path = drive.trajectorySequenceBuilder(startPose)
//                    .strafeRight(3)
//                    .forward(3)
//                    .splineTo(new Vector2d(14,-50), Math.toRadians(115))
//                    .splineTo(new Vector2d(5,-33), Math.toRadians(142))
//                    .addTemporalMarker(3, () -> {
////                        s.setTarget(-3400);
//                    })
//
//                    .forward(2)
//                    .addTemporalMarker(4.25,() ->{
//                        telemetry.update();
////                        rubberBandIntake.updatePower(-1);
//                    })
//                    .addTemporalMarker(4.75, ()-> {
//                        telemetry.update();
////                        rubberBandIntake.updatePower(0);
//                    })
//                    .waitSeconds(0.5)
//                    .back(10)
//
//
//                    .turn(Math.toRadians(-53))
//                    .back(14)
//                    .addDisplacementMarker(() ->{
////                        s.setTarget(0);
//                    })
//                    .strafeRight(15)
//                    .build();
//        } else {
//            path = drive.trajectorySequenceBuilder(startPose)
//                    .strafeRight(3)
//                    .forward(3)
//                    .splineTo(new Vector2d(14,-50), Math.toRadians(115))
//                    .splineTo(new Vector2d(5,-33), Math.toRadians(142))
//                    .addTemporalMarker(3, () -> {
////                        s.setTarget(-3400);
//                    })
//
//                    .forward(2)
//                    .addTemporalMarker(4.25,() ->{
//                        telemetry.update();
////                        rubberBandIntake.updatePower(-1);
//                    })
//                    .addTemporalMarker(4.75, ()-> {
//                        telemetry.update();
////                        rubberBandIntake.updatePower(0);
//                    })
//                    .waitSeconds(0.5)
//                    .back(10)
//                    .turn(Math.toRadians(-53))
//                    .addDisplacementMarker(() ->{
////                        s.setTarget(0);
//                    })
//                    .strafeRight(3)
//                    .build();
//        }
//
//        drive.followTrajectorySequenceAsync(path);
//    }
//}