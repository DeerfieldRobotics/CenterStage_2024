package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

@Autonomous
public class BlueRight extends OpMode {
    private final ColorDetectionPipeline colorDetection = new ColorDetectionPipeline("BLUE");
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
    private double secondBackboardYOffset = 0.0;


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

        outtake.setOuttakeKinematics(outtake.getOuttakeAngle()[0], 30, false);
        //outtake.update();
    }

    @Override
    public void init_loop() {
        purplePixelPath = colorDetection.getPosition();
        if(purplePixelPath.equals(ColorDetectionPipeline.StartingPosition.LEFT))
        {
            leftSpikeXOffest = -12.0;
            leftBackboardYOffset = 5.0;
            leftIntakeYOffset = -1.2;
            rightSpikeXOffset = 0.0;
            rightBackboardYOffset = 0.0;
            rightIntakeYOffset = 0.0;
            centerSpikeYOffset = 0.0;
            centerSpikeBackOffset = 0;
            secondBackboardYOffset = -7.0;
        }
        else if(purplePixelPath.equals(ColorDetectionPipeline.StartingPosition.CENTER))
        {
            leftSpikeXOffest = 0.0;
            leftBackboardYOffset = 0.0;
            leftIntakeYOffset = 0.0;
            rightSpikeXOffset = 0.0;
            rightIntakeYOffset = 0.0;
            rightBackboardYOffset = 0.0;
            centerSpikeYOffset = 7.3;
            centerSpikeBackOffset = 0.0;
            secondBackboardYOffset = -7.0;
        }
        else if(purplePixelPath.equals(ColorDetectionPipeline.StartingPosition.RIGHT))
        {
            leftSpikeXOffest = 0.0;
            leftBackboardYOffset = 0.0;
            leftIntakeYOffset = 0.0;
            rightSpikeXOffset = 3.0;
            rightBackboardYOffset = -7;
            rightIntakeYOffset = -.75;
            centerSpikeYOffset = 0.0;
            centerSpikeBackOffset = -8;
            secondBackboardYOffset = 4.0;
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
                .splineToSplineHeading(new Pose2d(23+leftSpikeXOffest+rightSpikeXOffset,-32+centerSpikeYOffset, Math.toRadians(180)), Math.toRadians(150))

                // TODO: SPIKE PURPLE
                .back(3.5-centerSpikeBackOffset)
                .addTemporalMarker(()->{
                    intake.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
                    intake.setMotorTargetPosition(250);
                    intake.setMotorPower(0.5);
                    intake.update();
                })
                // TODO: SLIDE DOWN, INTAKE CHANGE POSITION
                .build();

        drive.followTrajectorySequenceAsync(path);
    }
    @Override
    public void loop() {
//        avg = wp.getAvg();
        slide.update();
        drive.update();
//        telemetry.addLine(""+ slide.getPosition()[0]);
//        telemetry.addLine("AVG "+ avg);
//        telemetry.addLine("WHITE VALUES: "+cp.getWhiteVals());
        outtake.update();
        telemetry.update();
    }
}