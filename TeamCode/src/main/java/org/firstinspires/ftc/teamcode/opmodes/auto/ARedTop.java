package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.testers.PIDF;
import org.firstinspires.ftc.teamcode.utils.ColorDetectionPipelineJames;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "ARedTop")
public class ARedTop extends OpMode {
    private ColorDetectionPipelineJames colorDetection;
    private PIDF pidf;
    private SampleMecanumDrive drive;

    private TrajectorySequenceBuilder path;
    Pose2d start = new Pose2d(12,-63, Math.toRadians(90));

    private OpenCvCamera frontCamera;

    private int purplePixelPath;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);

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
        String p = colorDetection.getPosition();

        if (p == "RIGHT") purplePixelPath = 1;
        else if (p == "LEFT") purplePixelPath = -1;
        else purplePixelPath = 0;

        telemetry.addData("Position", colorDetection.getPosition());
        telemetry.addData("Color", colorDetection.getColor());
        telemetry.update();
    }

    @Override
    public void start() {
        // Temporary: move forward 3

        if (purplePixelPath == -1) {
            // Left

        } else if (purplePixelPath == 0) {
            // Center
            path.forward(32)
                    .back(5)
                    .setTangent(Math.toRadians(0))
                    .lineToLinearHeading(new Pose2d(50,-35,Math.toRadians(0)))
                    /*
                    .back(20)
                    .turn(Math.toRadians(180))
                     */
                    .lineToLinearHeading(new Pose2d(-30, -35,Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(-60, -35,Math.toRadians(180)))
                    .lineToLinearHeading(new Pose2d(50, -35,Math.toRadians(0)));

        } else if (purplePixelPath == 1) {
            // Right
        } else {

        }

        // Park
        path.back(5)
                .strafeLeft(10);

        drive.followTrajectorySequenceAsync(path.build());
    }

    @Override
    public void loop() {}
}
