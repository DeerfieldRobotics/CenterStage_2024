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
import org.firstinspires.ftc.teamcode.utils.OuttakeKotlin;
import org.firstinspires.ftc.teamcode.utils.SlideKotlin;
import org.firstinspires.ftc.teamcode.utils.WhiteDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "RedRight")
public class RedRight extends OpMode {
    private final ColorDetectionPipeline colorDetection = new ColorDetectionPipeline("RED");
    private final WhiteDetectionPipeline wp = new WhiteDetectionPipeline(20);
    private PIDF pidf;
    private SampleMecanumDrive drive;
    private IntakeKotlin intake;
    private OuttakeKotlin outtake;
    private SlideKotlin slide;
    private TrajectorySequence path;

    private double leftSpikeXOffest = 0.0;
    private double rightSpikeXOffset = 0.0;
    private double leftBackboardYOffset = 0.0;
    private double rightBackboardYOffset = 0.0;
    private double centerSpikeYOffset = 0.0;
    private double centerSpikeBackOffset = 0.0;
    private double leftIntakeYOffset = 0.0;
    private double rightIntakeYOffset = 0.0;


    private OpenCvCamera frontCamera;

    private ColorDetectionPipeline.StartingPosition purplePixelPath = ColorDetectionPipeline.StartingPosition.CENTER;

    private double avg = -1;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        slide = new SlideKotlin(hardwareMap);
        intake = new IntakeKotlin(hardwareMap);
        outtake = new OuttakeKotlin(hardwareMap, slide);
//        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        frontCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        frontCamera.setPipeline(colorDetection);
        frontCamera.setPipeline(colorDetection);
        frontCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                frontCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });

        outtake.setGateClosed(true);
        outtake.update();

        outtake.setOuttakeAngle(outtake.getOuttakeAngle()[0], 30, false);
        //outtake.update();
    }

    @Override
    public void init_loop() {
        purplePixelPath = colorDetection.getPosition();
        if(purplePixelPath.equals(ColorDetectionPipeline.StartingPosition.LEFT))
        {
            leftSpikeXOffest = -11.0;
            leftBackboardYOffset = 8.0;
            leftIntakeYOffset = -2.0;
            rightSpikeXOffset = 0.0;
            rightBackboardYOffset = 0.0;
            rightIntakeYOffset = 0.0;
            centerSpikeYOffset = 0.0;
            centerSpikeBackOffset = 0;
        }
        else if(purplePixelPath.equals(ColorDetectionPipeline.StartingPosition.CENTER))
        {
            leftSpikeXOffest = 0.0;
            leftBackboardYOffset = 0.0;
            leftIntakeYOffset = 0.0;
            rightSpikeXOffset = 0.0;
            rightIntakeYOffset = 0.0;
            rightBackboardYOffset = 0.0;
            centerSpikeYOffset = 5.8;
            centerSpikeBackOffset = 0.0;
        }
        else if(purplePixelPath.equals(ColorDetectionPipeline.StartingPosition.RIGHT))
        {
            leftSpikeXOffest = 0.0;
            leftBackboardYOffset = 0.0;
            leftIntakeYOffset = 0.0;
            rightSpikeXOffset = 5.0;
            rightBackboardYOffset = -8.0;
            rightIntakeYOffset = 2.0;
            centerSpikeYOffset = 0.0;
            centerSpikeBackOffset = -8;
        }

        telemetry.addLine(colorDetection.toString());
        telemetry.addData("Purple Pixel Path: ", purplePixelPath);
//        telemetry.addLine(""+wp.getAvg());
//        telemetry.addLine(wp.toString());
        telemetry.update();
    }

    @Override
    public void start() {
        intake.setServoPosition(IntakeKotlin.IntakePositions.INIT);
        intake.update();

        drive.setPoseEstimate(new Pose2d(11,-63, Math.toRadians(90)));
        path = drive.trajectorySequenceBuilder(new Pose2d(11,-63, Math.toRadians(90)))
                .setTangent(Math.toRadians(60))
                .addTemporalMarker(()->{
                    intake.setServoPosition(IntakeKotlin.IntakePositions.DRIVE);
                    intake.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    intake.update();
                })
                .splineToSplineHeading(new Pose2d(23+leftSpikeXOffest+rightSpikeXOffset,-30+centerSpikeYOffset, Math.toRadians(180)), Math.toRadians(150))

                // TODO: SPIKE PURPLE
                .back(2-centerSpikeBackOffset)
                .addTemporalMarker(()->{
                    intake.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intake.setMotorTargetPosition(250);
                    intake.setMotorPower(0.5);
                    intake.update();
                })
                .waitSeconds(0.5)
                .setTangent(0)
                .addTemporalMarker(()->{
                    intake.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    intake.setMotorPower(0);
                    intake.update();

                    slide.setTargetPosition(-1400);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(1);
                    slide.update();
                })
                // TODO: BRING SLIDE UP
                .splineToLinearHeading(new Pose2d(54,-35+leftBackboardYOffset+rightBackboardYOffset,Math.toRadians(180)), Math.toRadians(0))
                // TODO: OUTTAKE YELLOW
                .addTemporalMarker(()->{
                    outtake.setOuttakeExtended(true);
                    outtake.update();
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    slide.setTargetPosition(-400);
                    slide.update();
                })
                .waitSeconds(0.7)
                .back(2)
                .waitSeconds(0.3)
                .addTemporalMarker(()->{
                    outtake.setGateClosed(false);
                    outtake.update();
                })
                .waitSeconds(0.7)
                .addTemporalMarker(()-> {
                    slide.setTargetPosition(-1400);
                    slide.update();
                })
                .forward(10)
                .waitSeconds(0.5)
                .addTemporalMarker(()->{
                    outtake.setOuttakeExtended(false);
                    outtake.update();
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    slide.bottomOutProcedure();
                })
                // TODO: BRING SLIDE DOWN, RAISE INTAKE
                .setTangent(135)
                .splineToSplineHeading(new  Pose2d(24,-10, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new  Pose2d(50,-10, Math.toRadians(180)), Math.toRadians(180))
                .addTemporalMarker(()->{
                    intake.getIntakeServo().setPosition(0.775);
                    intake.update();
                })
                .splineToSplineHeading(new Pose2d(-57,-11, Math.toRadians(180)), Math.toRadians(180))
                // TODO: INTAKE 2 WHITE BOIS AND TRANSFER TO BOX
                .addTemporalMarker(()->{
                    intake.setMotorPower(0.8);
                    intake.update();
                })
                .waitSeconds(0.7)
                .addTemporalMarker(()->{
                    intake.setMotorPower(0);
                    intake.setServoPosition(IntakeKotlin.IntakePositions.TRANSFER);
                    intake.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    intake.update();
                    outtake.setTransferPosition(true);
                    outtake.update();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(()->{
                    outtake.update();
                    intake.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intake.setMotorTargetPosition(-240);
                    intake.setMotorPower(0.8);
                    intake.update();
                })
                .waitSeconds(0.7)
                .addTemporalMarker(()->{
                    intake.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    intake.setMotorPower(1);
                    intake.update();
                })
                .waitSeconds(0.7)
                .addTemporalMarker(()->{
                    intake.setMotorPower(0);
                    intake.setServoPosition(IntakeKotlin.IntakePositions.DRIVE);
                    intake.update();
                    outtake.setTransferPosition(false);
                    outtake.update();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()->{
                    outtake.setGateClosed(true);
                    outtake.update();
                })
                .lineToLinearHeading(new Pose2d(28, -10, Math.toRadians(180)))
                .addTemporalMarker(()->{
                    outtake.setTransferPosition(false);
                    outtake.update();
                    slide.setTargetPosition(-1400);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(1);
                    slide.update();

                })
                // TODO: SLIDE UP
                .setTangent(Math.toRadians(-45))
                .splineToConstantHeading(new Vector2d(53,-35), Math.toRadians(-45))
                // TODO: OUTTAKE 2 WHITE BOIS
//                .addTemporalMarker(()->{
//                    outtake.setOuttakeExtended(true);
//                    outtake.update();
//                })
                .waitSeconds(0.7)
                .addTemporalMarker(()->{
                    slide.setTargetPosition(-300);
                    slide.update();
                })
                .waitSeconds(0.7)
                .back(2)
                .waitSeconds(0.3)
                .addTemporalMarker(()->{
                    outtake.setGateClosed(false);
                    outtake.update();
                })
                .waitSeconds(0.7)
                .addTemporalMarker(()-> {
                    slide.setTargetPosition(-1400);
                    slide.update();
                })
                .waitSeconds(0.7)
//                .addTemporalMarker(()->{
//                    outtake.setOuttakeExtended(false);
//                    outtake.update();
//                })
                .waitSeconds(0.7)
                .addTemporalMarker(()->{
                    slide.bottomOutProcedure();
                })
                .forward(5)
                .strafeRight(30)
                .back(15)

                // TODO: SLIDE DOWN, INTAKE CHANGE POSITION
                .build();

        drive.followTrajectorySequenceAsync(path);
    }
    @Override
    public void loop() {
//        avg = wp.getAvg();
        drive.update();
//        telemetry.addLine(""+ slide.getPosition()[0]);
//        telemetry.addLine("AVG "+ avg);
//        telemetry.addLine("WHITE VALUES: "+cp.getWhiteVals());

        telemetry.update();
    }
}