package org.firstinspires.ftc.teamcode.opmodes.auto.meet1;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.testers.PIDF;
import org.firstinspires.ftc.teamcode.utils.ColorDetectionPipeline;
import org.firstinspires.ftc.teamcode.utils.IntakeKotlin;
import org.firstinspires.ftc.teamcode.utils.SlideKotlin;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name = "ARedTop")
public class ARedTop extends OpMode {
    private final ColorDetectionPipeline colorDetection = new ColorDetectionPipeline();
    private PIDF pidf;
    private SampleMecanumDrive drive;
    private IntakeKotlin intake;
    private SlideKotlin slide;
    private TrajectorySequenceBuilder path;
    Pose2d start = new Pose2d(12,-63, Math.toRadians(90));

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

        path = drive.trajectorySequenceBuilder(start);
    }

    @Override
    public void init_loop() {
        purplePixelPath = colorDetection.getPosition();

        telemetry.addLine(colorDetection.toString());
        telemetry.update();
    }

    @Override
    public void start() {
        // Temporary: move forward 3

        if (purplePixelPath == ColorDetectionPipeline.StartingPosition.LEFT) {
            // Left

        } else if (purplePixelPath == ColorDetectionPipeline.StartingPosition.CENTER) {
            // Center
            path.forward(32)
                    .waitSeconds(1)
                    .back(5)
                    .setTangent(Math.toRadians(0))
                    .lineToLinearHeading(new Pose2d(50,-35,Math.toRadians(0)))
                    .waitSeconds(2)
                    /*
                    .back(20)
                    .turn(Math.toRadians(180))
                     */
                    .lineToLinearHeading(new Pose2d(-30, -35,Math.toRadians(0)))
                    .waitSeconds(2)
                    .lineToLinearHeading(new Pose2d(-60, -35,Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(50, -35,Math.toRadians(0)));

        } else if (purplePixelPath == ColorDetectionPipeline.StartingPosition.RIGHT) {
            // Right
        } else {

        }

        // Park
        path.back(5)
                .strafeLeft(10);

        drive.followTrajectorySequenceAsync(path.build());
    }
    @Override
    public void loop() {
        drive.update();
    }
}
