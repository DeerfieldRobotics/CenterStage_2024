package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
    private final ColorDetectionPipeline colorDetection = new ColorDetectionPipeline();
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
        start = new Pose2d(12,-63, Math.toRadians(90));
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
                .splineToLinearHeading(new Pose2d(12-mult*4,-34, Math.toRadians(180-45*mult)), Math.toRadians(180-45*mult)) //drop off purple
                //TODO: OUTTAKE PURPLE HERE
                .addTemporalMarker(2,()->{
                    intake.intake(0);
                })
                .setTangent(Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(54,-34+mult * 7,Math.toRadians(180))) //drop off yellow
                .waitSeconds(2)
                //TODO: OUTTAKE YELLOW HERE, BRING SLIDE UP AND OUTTAKE
                .addTemporalMarker(3, ()->{
                    slide.setTargetPosition(-1050);
                    //TODO: USE THE OVERLOADED METHOD FROM IntakeKotlin.kt FOR intakeProcedure WHICH RUNS IT ASYNCHRONOUSLY ALLEGEDLY LINE 116 of IntakeKotlin.kt (intakeProcedure (toggle: Boolean, target: Int))

                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(1);

                })
                .addTemporalMarker(3.5, ()->{
                    intake.armToggle();
                })

                .addTemporalMarker(3.75, ()->{
                    intake.getOuttakeServo().setPosition(0.34); //TODO change to intake.outtakeToggle(true) if possible
                })
                .addTemporalMarker(4.0, ()->{
                    intake.armToggle();
//                    slide.bottomOut();
//                    slide.setPower(-1);
                })
                .addTemporalMarker(4.5,()->{
//                    intake.getOuttakeServo().setPosition(0.34);
                    slide.setTargetPosition(0);
                    slide.setPower(-1);

//                    slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                /*
                .back(20)
                .turn(Math.toRadians(180))
                 */
//                                .lineToLinearHeading(new Pose2d(-30, -35,Math.toRadians(0)))
                .strafeRight(-2*mult+0.01)
                .lineToLinearHeading(new Pose2d(-60, -35,Math.toRadians(180))) //Go back to stack
                .addTemporalMarker(7,()->{
                    intake.intake(1);
                })
                .addTemporalMarker(9,()->{
                    intake.intake(0);
                })
                //TODO: ADD INTAKE HERE
                .lineToLinearHeading(new Pose2d(50, -35,Math.toRadians(180))) //drop off pixel
                //TODO: OUTTAKE PIXEL HERE
                .forward(5)
                .addTemporalMarker(13, ()->{
                    intake.getOuttakeServo().setPosition(0);
                    slide.setTargetPosition(-1150);
                    //TODO: USE THE OVERLOADED METHOD FROM IntakeKotlin.kt FOR intakeProcedure WHICH RUNS IT ASYNCHRONOUSLY ALLEGEDLY LINE 116 of IntakeKotlin.kt (intakeProcedure (toggle: Boolean, target: Int))

                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(1);

                })
                .addTemporalMarker(13.5, ()->{
                    intake.armToggle();
                })
//                .addTemporalMarker(13.75, ()->{
//                    intake.getOuttakeServo().setPosition(0.34);
//                })
//
//                .addTemporalMarker(14.5,()->{
////                    intake.getOuttakeServo().setPosition(0.34);
//                    slide.setTargetPosition(0);
//                    slide.setPower(-1);
//
////                    slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                })
                .strafeRight(10)
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
