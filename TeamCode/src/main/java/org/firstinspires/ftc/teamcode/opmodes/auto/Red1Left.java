package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.graphics.Color;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.testers.PIDF;
import org.firstinspires.ftc.teamcode.utils.ColorDetectionPipeline;
import org.firstinspires.ftc.teamcode.utils.IntakeKotlin;
import org.firstinspires.ftc.teamcode.utils.SlideKotlin;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "ARedTopLeft")
public class Red1Left extends OpMode {
    private final ColorDetectionPipeline colorDetection = new ColorDetectionPipeline("RED");
    private PIDF pidf;
    private SampleMecanumDrive drive;
    private IntakeKotlin intake;
    private SlideKotlin slide;
    private TrajectorySequence path;
    Pose2d start;
    private double mult = 0;
    private double centerx = 0;

    private OpenCvCamera frontCamera;

    private ColorDetectionPipeline.StartingPosition purplePixelPath;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        slide = new SlideKotlin(hardwareMap);
        intake = new IntakeKotlin(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        frontCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        frontCamera.setPipeline(colorDetection);
        frontCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                frontCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });
        start = new Pose2d(-39.75,-63, Math.toRadians(90));
    }

    @Override
    public void init_loop() {
        purplePixelPath = colorDetection.getPosition();

        if(purplePixelPath.equals(ColorDetectionPipeline.StartingPosition.CENTER)) {
            mult =  0.0;
            centerx = -5;
        }
        else if (purplePixelPath.equals(ColorDetectionPipeline.StartingPosition.LEFT)) mult = 1.0;
        else mult = -1.0;

        telemetry.addLine(colorDetection.toString());
        telemetry.update();
    }

    @Override
    public void start() {
        // Temporary: move forward 3
        drive.setPoseEstimate(start);

        path = drive.trajectorySequenceBuilder(start)
                .strafeRight(5)
                .waitSeconds(0.05)
                .splineToSplineHeading(new Pose2d(-36.5+centerx-4.5*mult,-36+(4-4*Math.abs(mult)),Math.toRadians(90+53*mult)), Math.toRadians(90+53*mult)) //drop off purple
                .addTemporalMarker(()->{
                    intake.getIntakeServo().setPosition(0.9);
                })
                .back(2)
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-44.5, -45.5, Math.toRadians(180)), Math.toRadians(225))
                .splineToLinearHeading(new Pose2d(-57,-15, Math.toRadians(180)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-61,-12, Math.toRadians(180)), Math.toRadians(180))
                //INTAKE
                .addTemporalMarker(()->{
                    intake.intake(0.35);
                    intake.getIntakeServo().setPosition(0.8);
                })
                .waitSeconds(0.6)
                .addTemporalMarker(()->{
                    intake.intake(0.0);
                })
                .back(9)
                .addTemporalMarker(()->{
                    intake.getIntakeServo().setPosition(0.0);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(()->{
                    intake.intake(0.7);
                })
                .waitSeconds(1.3)
                .addTemporalMarker(()->{
                    intake.intake(0.0);
                   // intake.outtakeToggle();
                    intake.getIntakeServo().setPosition(1.0);
                })
                .lineToLinearHeading(new Pose2d(28, -12, Math.toRadians(180)))
                .addTemporalMarker(()->{
                    slide.setTargetPosition(-1050);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(-1);
                })
                .splineToConstantHeading(new Vector2d(53.25,-32+7.5*mult), Math.toRadians(-45))
                //OUTTAKE 2
                .addTemporalMarker(()->{
                   // intake.armToggle();
                })
                .waitSeconds(0.4)
                .addTemporalMarker(()->{
                   // intake.getOuttakeServo().setPosition(0.34);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()->{
                    slide.setTargetPosition(-1300);
                })
                .waitSeconds(0.4)
                .addTemporalMarker(()->{
                   // intake.armToggle();
                })
                .waitSeconds(0.4)
                .addTemporalMarker(()->{
                    slide.setTargetPosition(0);
                    slide.setPower(1);
                })
                //PARK
                .forward(5)
                .strafeRight(27)
                .back(10)
                .waitSeconds(10)
                .build();

//        if (purplePixelPath == ColorDetectionPipeline.StartingPosition.LEFT) {
//            // Left
//
//        } else if (purplePixelPath == ColorDetectionPipeline.StartingPosition.CENTER) {
//            // Center
//            path.forward(32)
//                    .back(5)
//                    .setTangent(Math.toRadians(0))
//                    .lineToLinearHeading(new Pose2d(50,-35,Math.toRadians(0)))
//                    /*
//                    .back(20)
//                    .turn(Math.toRadians(180))
//                     */
//                    .lineToLinearHeading(new Pose2d(-30, -35,Math.toRadians(0)))
//                    .lineToLinearHeading(new Pose2d(-60, -35,Math.toRadians(180)))
//                    .lineToLinearHeading(new Pose2d(50, -35,Math.toRadians(0)));
//
//        } else if (purplePixelPath == ColorDetectionPipeline.StartingPosition.RIGHT) {
//            // Right
//        } else {
//
//        }
//
//        // Park
//        path.back(5)
//                .strafeLeft(10);

        drive.followTrajectorySequenceAsync(path);
    }
    @Override
    public void loop() {
        drive.update();
    }
}
