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
            leftBackboardYOffset = 10.0;
            rightSpikeXOffset = 0.0;
            rightBackboardYOffset = 0.0;
            centerSpikeYOffset = 0.0;
            centerSpikeBackOffset = 0;
        }
        else if(purplePixelPath.equals(ColorDetectionPipeline.StartingPosition.CENTER))
        {
            leftSpikeXOffest = 0.0;
            leftBackboardYOffset = 0.0;
            rightSpikeXOffset = 0.0;
            rightBackboardYOffset = 0.0;
            centerSpikeYOffset = 5.8;
            centerSpikeBackOffset = 0.0;
        }
        else if(purplePixelPath.equals(ColorDetectionPipeline.StartingPosition.RIGHT))
        {
            leftSpikeXOffest = 0.0;
            leftBackboardYOffset = 0.0;
            rightSpikeXOffset = 5.0;
            rightBackboardYOffset = -4.0;
            centerSpikeYOffset = 0.0;
            centerSpikeBackOffset = -5;
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
                .waitSeconds(1.0)
                .setTangent(0)
                .addTemporalMarker(()->{
                    intake.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    intake.setMotorPower(0);
                    intake.update();

                    slide.setTargetPosition(-1400);
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide.setPower(0.7);
                    slide.update();
                })
                // TODO: BRING SLIDE UP
                .splineToLinearHeading(new Pose2d(54,-37+leftBackboardYOffset+rightBackboardYOffset,Math.toRadians(180)), Math.toRadians(0))
                // TODO: OUTTAKE YELLOW
                .addTemporalMarker(()->{
                    outtake.setOuttakeExtended(true);
                    outtake.update();
                })
                .waitSeconds(2)
                .addTemporalMarker(()->{
                    slide.setTargetPosition(-400);
                    slide.update();
                })
                .waitSeconds(1)
                .back(2)
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    outtake.setGateClosed(false);
                    outtake.update();
                })
                .waitSeconds(2)
                .addTemporalMarker(()-> {
                    slide.setTargetPosition(-1400);
                    slide.update();
                })
                .waitSeconds(2)
                .addTemporalMarker(()->{
                    outtake.setOuttakeExtended(false);
                    outtake.update();
                })
                .waitSeconds(2)
                .setTangent(135)
                .addTemporalMarker(()->{
                    slide.setTargetPosition(0);
                    slide.setPower(1);
                    slide.update();
                })
                // TODO: BRING SLIDE DOWN, RAISE INTAKE
                .splineToSplineHeading(new  Pose2d(24,-10, Math.toRadians(170)), Math.toRadians(180))
                .addTemporalMarker(()->{
                    slide.setPower(0);
                    slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    slide.update();
                })
                .waitSeconds(0.5)
                // TODO: INTAKE 2 WHITE BOIS AND TRANSFER TO BOX
                .splineToSplineHeading(new Pose2d(-55,-9, Math.toRadians(170)), Math.toRadians(180))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(28, -10, Math.toRadians(170)))
                // TODO: SLIDE UP
                .splineToConstantHeading(new Vector2d(50,-37), Math.toRadians(-45))
                // TODO: OUTTAKE 2 WHITE BOIS

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