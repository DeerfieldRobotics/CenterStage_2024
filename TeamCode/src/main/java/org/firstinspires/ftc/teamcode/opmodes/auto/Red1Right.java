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

@Autonomous(name = "ARedTopWorks")
public class Red1Right extends OpMode {
    private final ColorDetectionPipeline colorDetection = new ColorDetectionPipeline("RED");
    private PIDF pidf;
    private SampleMecanumDrive drive;
    private IntakeKotlin intake;
    private SlideKotlin slide;
    private TrajectorySequence path;
    private double mult = 0.0;

    private OpenCvCamera frontCamera;

    private ColorDetectionPipeline.StartingPosition purplePixelPath;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        slide = new SlideKotlin(hardwareMap);
        intake = new IntakeKotlin(hardwareMap, slide);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
    }

    @Override
    public void init_loop() {
        purplePixelPath = colorDetection.getPosition();


        if(purplePixelPath.equals(ColorDetectionPipeline.StartingPosition.CENTER)) mult =  0.0;
        else if (purplePixelPath.equals(ColorDetectionPipeline.StartingPosition.LEFT)) mult = 1.0;
        else mult = -1.0;

        telemetry.addLine(colorDetection.toString());
        telemetry.update();
    }

    @Override
    public void start() {
        drive.setPoseEstimate(new Pose2d(5,-63, Math.toRadians(90)));

        path = drive.trajectorySequenceBuilder(new Pose2d(5,-63, Math.toRadians(90)))
                .splineToSplineHeading(new Pose2d(10-mult*3,-34+(1-Math.abs(mult)), Math.toRadians(90+52*mult)), Math.toRadians(90+52*mult)) //drop off purple
                .addTemporalMarker(() -> {
                    intake.intakeServo(1);
                })
                .back(5)
                .splineToLinearHeading(new Pose2d(19, -48, Math.toRadians(180)), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                //.strafeRight(14)
                .lineToLinearHeading(new Pose2d(50,-35+mult*7,Math.toRadians(180)))
                .addTemporalMarker(()->{//3.0
                    slide.setTargetPosition(-1000);
                    slide.setPower(-1);

                })
                .addTemporalMarker(()->{//3.3
                    intake.armToggle();
                })
                //.splineToSplineHeading(new Pose2d(50,-35+mult*7,Math.toRadians(180)), Math.toRadians(0))
                .waitSeconds(2.0)
                //TODO: OUTTAKE YELLOW HERE, BRING SLIDE UP AND OUTTAKE
                .addTemporalMarker(()->{//4.5
                    intake.getOuttakeServo().setPosition(0.34);
                })
                .addTemporalMarker(()->{//4.8
                    slide.setTargetPosition(-1050);
                    slide.setPower(-1);
                 })
                .setTangent(135)
                .splineToConstantHeading(new Vector2d(24,-11.8), Math.toRadians(180))
                .addTemporalMarker(()->{//5.0
                    intake.armToggle();
                })
                .addTemporalMarker(()->{//5.5
                    //Bottom out

                    slide.setTargetPosition(0);
                    slide.setPower(1);

                    intake.getIntakeServo().setPosition(1);
                })
//                .setTangent(180)
                .splineToConstantHeading(new Vector2d(-66,-11.8), Math.toRadians(180))
                .addTemporalMarker(()->{//11
                    intake.getIntakeMotor().setPower(0.6);
                })
                .waitSeconds(1.0)
                .addTemporalMarker(()->{//11.5
                    intake.getIntakeMotor().setPower(0.0);
                })
                .back(10)
                .waitSeconds(1.0)
                .addTemporalMarker(()->{//12
                    intake.getIntakeServo().setPosition(0);
                    intake.getIntakeMotor().setPower(0.6);
                })
                .addTemporalMarker(()->{//13
                    intake.getIntakeMotor().setPower(0.0);
                    intake.getOuttakeServo().setPosition(0.0);
                })
                .lineToLinearHeading(new Pose2d(28, -11.8, Math.toRadians(180)))
//                .setTangent(-45)
                .addTemporalMarker(()->{//19
                    slide.setTargetPosition(-1075);
                    slide.setPower(-1);
                })
                .splineToConstantHeading(new Vector2d(50,-37), Math.toRadians(-45))
                .addTemporalMarker(()->{//20
                    intake.armToggle();
                })
                .addTemporalMarker(()->{//21
                    intake.getOuttakeServo().setPosition(0.34);
                })
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
        telemetry.addLine(""+ slide.getPosition()[0]);
        telemetry.update();
    }
}
