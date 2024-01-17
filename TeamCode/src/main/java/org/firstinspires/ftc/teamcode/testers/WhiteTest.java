package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.detection.WhiteDetectionPipeline;
import org.firstinspires.ftc.teamcode.roadrunner.drive.CogchampDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "WhiteTest", group = "b")
@Config
public class WhiteTest extends OpMode {
    private OpenCvCamera frontCamera;
    private WhiteDetectionPipeline whiteDetection;
    private double timeToAlign = 0;
    CogchampDrive drive = new CogchampDrive(hardwareMap);
    public static double kP = 0.001;
    public static double kI = 0;
    public static double kD = 0;
    @Override
    public void init() {
        whiteDetection = new WhiteDetectionPipeline();

        whiteDetection.setTarget(160);
        whiteDetection.getController().setPID(kP,kI,kD);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        frontCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);

        frontCamera.setPipeline(whiteDetection);
        frontCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() { frontCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT); }
            @Override
            public void onError(int errorCode) { }
        });
    }

    @Override
    public void init_loop() {
        double avg = whiteDetection.getPosition();
        telemetry.addLine(String.valueOf(avg));
        double error = whiteDetection.getTarget()- avg;
        telemetry.addLine(String.valueOf(error));
        telemetry.update();
    }
    @Override
    public void start() {

    }
    @Override
    public void loop() {
        whiteDetection.getController().setPID(kP,kI,kD);
        double avg = whiteDetection.getPosition();
        telemetry.addData("Position", avg);
        double error = whiteDetection.getTarget()- avg;
        telemetry.addData("Error", error);
        telemetry.addData("Robot Aligned", whiteDetection.robotAligned());
        telemetry.addData("Time to Align", timeToAlign == 0 ? time : timeToAlign);
        if(whiteDetection.robotAligned() && timeToAlign == 0) {
            timeToAlign = time;
        }

        telemetry.update();
        whiteDetection.alignRobot(drive);
    }
}