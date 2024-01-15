package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utils.detection.WhiteDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "WhiteTestPhone")
public class WhiteTestPhone extends OpMode {
    private final WhiteDetectionPipeline colorDetection = new WhiteDetectionPipeline(null);
//    private OpenCvInternalCamera frontCamera;
    private OpenCvCamera frontCamera;
    private double centerx = 0;
    private double mult = 0.0;

//    private SampleMecanumDrive drive;
    private double stackOffset = 0;

//    private ColorDetectionPipeline.StartingPosition purplePixelPath;
//    private ColorDetectionPipeline cp = new ColorDetectionPipeline("WHITE");

    private double avg = -1;

    @Override
    public void init() {
//        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

//        frontCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        frontCamera = OpenCvCameraFactory.getInstance()
                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);



        frontCamera.setPipeline(colorDetection);
        frontCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                frontCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    @Override
    public void init_loop() {
//        purplePixelPath = colorDetection.getPosition();

        avg = colorDetection.getPosition();

        telemetry.addLine(String.valueOf(avg));

        telemetry.update();
    }

    @Override
    public void start() {
//        drive.
    }
    @Override
    public void loop() {
        avg = colorDetection.getPosition();

        telemetry.addLine(String.valueOf(avg));

        telemetry.addLine(colorDetection.toString());

        telemetry.update();

    }
}
