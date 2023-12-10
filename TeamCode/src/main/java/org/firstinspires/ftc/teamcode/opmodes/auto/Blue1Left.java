package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.testers.PIDF;
import org.firstinspires.ftc.teamcode.utils.ColorDetectionPipeline;
import org.firstinspires.ftc.teamcode.utils.IntakeKotlin;
import org.firstinspires.ftc.teamcode.utils.SlideKotlin;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "BlueClose")
public class Blue1Left extends OpMode {
    private final ColorDetectionPipeline colorDetection = new ColorDetectionPipeline("BLUE");
    private PIDF pidf;
    private SampleMecanumDrive drive;
    private IntakeKotlin intake;
    private SlideKotlin slide;
    private TrajectorySequence path;
    private double mult = 0.0;
    private double righty = 0.0;
    private double lefty = 0.0;
    private double centery = 0.0;
    private double rightBackboard = 0.0;
    private double centerBackboard = 0.0;
    private double leftConst = 0.0;
    private double centerx = 0;
    private double angleChange = 0.0;
    private double leftChange = 0.0;

    private OpenCvCamera frontCamera;

    private ColorDetectionPipeline.StartingPosition purplePixelPath;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        slide = new SlideKotlin(hardwareMap);
        intake = new IntakeKotlin(hardwareMap, slide);
//        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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


        if(purplePixelPath.equals(ColorDetectionPipeline.StartingPosition.CENTER)) {
            mult = 0.0;
            centerx = 5.6;
            centerBackboard = -0.5;
            righty = 0.0;
            rightBackboard = 0;
            lefty = 0.0;
            centery = 3.5;
            leftConst = 0;
            angleChange = 0.0;
            leftChange = 0;
        }
        else if (purplePixelPath.equals(ColorDetectionPipeline.StartingPosition.LEFT)) {
            mult = -1.0;
            righty = 0.0;
            rightBackboard = -1.0;
            centerx = 0.0;
            centerBackboard = 0.0;
            lefty = 2.75;
            centery = -1.0;
            leftConst = 1.0;
            angleChange = -10;
            leftChange = -1;
        }
        else {
            mult = 1.0;
            righty = -0.3;
            rightBackboard = 0.5;
            centerx = 0.0;
            centerBackboard = 0.0;
            lefty = 0.0;
            centery = 0.0;
            leftConst = -0.5;
            angleChange = 0;
            leftChange = 0;
        }

        telemetry.addLine(colorDetection.toString());
        telemetry.update();
    }

    @Override
    public void start() {
        frontCamera.closeCameraDevice();
        drive.setPoseEstimate(new Pose2d(16.5,63, Math.toRadians(270)));
        path = drive.trajectorySequenceBuilder(new Pose2d(16.5,63, Math.toRadians(270)))
                .addTemporalMarker(3.5, ()->{
                    slide.setTargetPosition(-1000);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(-1);
                })
                .strafeRight(5)
                .waitSeconds(0.05)
                .setTangent(-45)
                .splineToSplineHeading(new Pose2d(11.5-4.5*mult+centerx+leftConst,36-(4-4*Math.abs(mult)),Math.toRadians(270-53*mult)), Math.toRadians(270-53*mult)) //drop off purple
                .addTemporalMarker(()->{
                    intake.getIntakeServo().setPosition(0.9);
                })
                .back(4)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(53.5,33+rightBackboard+centerBackboard-7*mult,Math.toRadians(180)), Math.toRadians(-60))
                .addTemporalMarker(()->{
                    //intake.armToggle();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(()->{
                 //   intake.getOuttakeServo().setPosition(0.34);
                })
                .waitSeconds(0.4)
                .addTemporalMarker(()->{
                    slide.setTargetPosition(-1200);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(()->{
                   // intake.armToggle();
                })
                .waitSeconds(0.4)
                .addTemporalMarker(()->{
                    slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    slide.setPower(1.0);
                })
                //TODO: OUTTAKE YELLOW HERE, BRING SLIDE UP AND OUTTAKE

                .setTangent(135)
                .splineToConstantHeading(new Vector2d(24,8.5), Math.toRadians(180))
                .addTemporalMarker(()->{
                    slide.setPower(0);
                    slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                })
                .splineToLinearHeading(new Pose2d(-60.25,20+0.4*mult+righty+lefty+centery, Math.toRadians(180+angleChange)), Math.toRadians(180))
                //INTAKE
                .addTemporalMarker(()->{
                    intake.intake(0.35);
                })
                .addTemporalMarker(()->{
                    intake.getIntakeServo().setPosition(0.85);
                })
                .waitSeconds(0.8)
                .addTemporalMarker(()->{
                    intake.intake(0.0);
//                    intake.getIntakeServo().setPosition(1.0);
                })
                .back(6)
                .addTemporalMarker(()->{
                    intake.getIntakeServo().setPosition(0.0);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(()->{
                    intake.intake(0.7);
                })
                .waitSeconds(1.5)
                .addTemporalMarker(()->{
                    intake.intake(-0.1);
              //      intake.outtakeToggle();
                    intake.getIntakeServo().setPosition(1.0);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()->{
                    intake.intake(0.0);
                })
                .lineToLinearHeading(new Pose2d(28, 8.5, Math.toRadians(180)))
                .addTemporalMarker(()->{
                    slide.setTargetPosition(-1600);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(-1);
                })
                .splineToConstantHeading(new Vector2d(53.5+leftChange,36), Math.toRadians(45))
                //OUTTAKE 2
                .addTemporalMarker(()->{
             //       intake.armToggle();
                })
                .waitSeconds(0.4)
                .addTemporalMarker(()->{
                //    intake.getOuttakeServo().setPosition(0.34);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()->{
                    slide.setTargetPosition(-1700);
                })
                .waitSeconds(0.4)
                .addTemporalMarker(()->{
                //    intake.armToggle();
                })
                .waitSeconds(0.4)
                .addTemporalMarker(()->{
                    slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    slide.setPower(1.0);
                })
                //PARK
                .forward(8)
                .addTemporalMarker(()->{
                    slide.setPower(0);
                    slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                })
                .strafeLeft(27)
                .back(10)
                .waitSeconds(10)
                .build();
        drive.followTrajectorySequenceAsync(path);
    }
    @Override
    public void loop() {

        drive.update();
        telemetry.addLine(""+ slide.getPosition()[0]);
        telemetry.update();
    }
}
