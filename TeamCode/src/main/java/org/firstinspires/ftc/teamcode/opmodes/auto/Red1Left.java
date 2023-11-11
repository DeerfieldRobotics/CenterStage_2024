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
    private int mult = 0;

    private OpenCvCamera frontCamera;

    private ColorDetectionPipeline.StartingPosition purplePixelPath;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        slide = new SlideKotlin(hardwareMap);
        intake = new IntakeKotlin(hardwareMap, slide);

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
        start = new Pose2d(-38,-63, Math.toRadians(90));
    }

    @Override
    public void init_loop() {
        purplePixelPath = colorDetection.getPosition();


        if(purplePixelPath.equals(ColorDetectionPipeline.StartingPosition.CENTER)) mult =  0;
        else if (purplePixelPath.equals(ColorDetectionPipeline.StartingPosition.LEFT)) mult = 1;
        else mult = -1;

        telemetry.addLine(colorDetection.toString());
        telemetry.update();
    }

    @Override
    public void start() {
        // Temporary: move forward 3
        drive.setPoseEstimate(start);

        path = drive.trajectorySequenceBuilder(start)
                .splineToLinearHeading(new Pose2d(-35-mult*3,-34+(1-Math.abs(mult)), Math.toRadians(90+52*mult)), Math.toRadians(90)) //drop off purple
                //OUTTAKE PURPLE HERE
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-55,-55, Math.toRadians(180)), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-55,-11.8, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-58, -11.8, Math.toRadians(180)))
                //INTAKE
                .addDisplacementMarker(()->{
                    intake.intake(1);
                })
                .waitSeconds(2)
                .addDisplacementMarker(()->{
                    intake.intake(0);
                })
                .lineToLinearHeading(new Pose2d(35, -11.8, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(52, -35+mult*7,Math.toRadians(180)))
                .addTemporalMarker(13.7, ()->{
                    slide.setTargetPosition(-1050);
                    //TODO: USE THE OVERLOADED METHOD FROM IntakeKotlin.kt FOR intakeProcedure WHICH RUNS IT ASYNCHRONOUSLY ALLEGEDLY LINE 116 of IntakeKotlin.kt (intakeProcedure (toggle: Boolean, target: Int))

                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(1);

                })
                .addTemporalMarker(14.2, ()->{
                    intake.armToggle();
                })

                .addTemporalMarker(14.5, ()->{
                    intake.getOuttakeServo().setPosition(0.34); //TODO change to intake.outtakeToggle(true) if possible
                })
                .addTemporalMarker(14.9, ()->{
                    intake.armToggle();
                })
                .addTemporalMarker(15.3,()->{
                    slide.setTargetPosition(0);
                    slide.setPower(-1);
                })
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(35, -11.8, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-58, -11.8, Math.toRadians(180)))
                //TODO INTAKE NEW 2
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(35, -11.8, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(52, -35,Math.toRadians(180)))
                //TODO OUTTAKE 2
                .addTemporalMarker(28.0, ()->{
                    slide.setTargetPosition(-1050);
                    //TODO: USE THE OVERLOADED METHOD FROM IntakeKotlin.kt FOR intakeProcedure WHICH RUNS IT ASYNCHRONOUSLY ALLEGEDLY LINE 116 of IntakeKotlin.kt (intakeProcedure (toggle: Boolean, target: Int))

                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(1);

                })
                .addTemporalMarker(28.4, ()->{
                    intake.armToggle();
                })

                .addTemporalMarker(28.7, ()->{
                    intake.getOuttakeServo().setPosition(0.34); //TODO change to intake.outtakeToggle(true) if possible
                })
                .addTemporalMarker(29.1, ()->{
                    intake.armToggle();
                })
                .addTemporalMarker(29.5,()->{
                    slide.setTargetPosition(0);
                    slide.setPower(-1);
                })
                .waitSeconds(20)


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
