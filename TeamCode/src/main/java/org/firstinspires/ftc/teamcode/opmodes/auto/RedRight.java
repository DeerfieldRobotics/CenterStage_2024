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
    private OuttakeKotlin outake;
    private SlideKotlin slide;
    private TrajectorySequence path;

    private double leftSpikeXOffest = 0.0;
    private double rightSpikeXOffset = 0.0;
    private double leftBackboardXOffset = 0.0;
    private double rightBackboardXOffset = 0.0;
    private double centerSpikeYOffset = 0.0;


    private OpenCvCamera frontCamera;

    private ColorDetectionPipeline.StartingPosition purplePixelPath = ColorDetectionPipeline.StartingPosition.CENTER;

    private double avg = -1;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
//        slide = new SlideKotlin(hardwareMap);
        intake = new IntakeKotlin(hardwareMap);
//        outake = new OuttakeKotlin(hardwareMap, slide);
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
    }

    @Override
    public void init_loop() {
        purplePixelPath = colorDetection.getPosition();
        if(purplePixelPath.equals(ColorDetectionPipeline.StartingPosition.LEFT))
        {
            leftSpikeXOffest = 0.0;
            leftBackboardXOffset = 0.0;
            rightSpikeXOffset = 0.0;
            rightBackboardXOffset = 0.0;
            centerSpikeYOffset = 0.0;
        }
        else if(purplePixelPath.equals(ColorDetectionPipeline.StartingPosition.CENTER))
        {
            leftSpikeXOffest = 0.0;
            leftBackboardXOffset = 0.0;
            rightSpikeXOffset = 0.0;
            rightBackboardXOffset = 0.0;
            centerSpikeYOffset = 0.0;
        }
        else if(purplePixelPath.equals(ColorDetectionPipeline.StartingPosition.RIGHT))
        {
            leftSpikeXOffest = 0.0;
            leftBackboardXOffset = 0.0;
            rightSpikeXOffset = 0.0;
            rightBackboardXOffset = 0.0;
            centerSpikeYOffset = 0.0;
        }

//        telemetry.addLine(colorDetection.toString());
        telemetry.addData("Purple Pixel Path: ", purplePixelPath);
//        telemetry.addLine(""+wp.getAvg());
//        telemetry.addLine(wp.toString());
        telemetry.update();
    }

    @Override
    public void start() {
        drive.setPoseEstimate(new Pose2d(11,-63, Math.toRadians(90)));
        path = drive.trajectorySequenceBuilder(new Pose2d(11,-63, Math.toRadians(90)))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(9+leftSpikeXOffest+rightSpikeXOffset,-37+centerSpikeYOffset), Math.toRadians(120))

                // TODO: SPIKE PURPLE
                .addTemporalMarker(()->{
                    intake.getIntakeMotor().setPower(-1);
                })
                .waitSeconds(0.7)
                .addTemporalMarker(()->{
                    intake.getIntakeMotor().setPower(0);
//                    slide.setTargetPosition(-1000);
                })
                .setTangent(0)
                .addTemporalMarker(()->{
//                    slide.setPower(1);
                })
                // TODO: BRING SLIDE UP
                .splineToLinearHeading(new Pose2d(50+leftBackboardXOffset+rightBackboardXOffset,-35,Math.toRadians(180)), Math.toRadians(0))
                .addTemporalMarker(()->{
//                    outtake.getWristServo().setPosition();
                })
                .waitSeconds(1.0)
                // TODO: OUTTAKE YELLOW
                .setTangent(135)
                // TODO: BRING SLIDE DOWN, RAISE INTAKE
                .splineToConstantHeading(new Vector2d(24,-11.8), Math.toRadians(180))
                // TODO: INTAKE 2 WHITE BOIS AND TRANSFER TO BOX

                .splineToConstantHeading(new Vector2d(-66,-11.8), Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(28, -11.8, Math.toRadians(180)))
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
//        drive.update();
//        telemetry.addLine(""+ slide.getPosition()[0]);
//        telemetry.addLine("AVG "+ avg);
//        telemetry.addLine("WHITE VALUES: "+cp.getWhiteVals());

        telemetry.update();
    }
}
