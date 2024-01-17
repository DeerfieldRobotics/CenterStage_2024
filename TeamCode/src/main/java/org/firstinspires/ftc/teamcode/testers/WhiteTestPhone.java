package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utils.detection.WhiteDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "WhiteTestPhone", group = "b")
public class WhiteTestPhone extends OpMode {
    private final WhiteDetectionPipeline colorDetection = new WhiteDetectionPipeline();
    private OpenCvInternalCamera frontCamera;
    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        frontCamera = OpenCvCameraFactory.getInstance()
                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        frontCamera.setPipeline(colorDetection);
        frontCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() { frontCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT); }
            @Override
            public void onError(int errorCode) { }
        });
    }

    @Override
    public void init_loop() {
        double avg = colorDetection.getPosition();
        telemetry.addLine(String.valueOf(avg));
        telemetry.update();
    }

    @Override
    public void start() { }
    @Override
    public void loop() {
        double avg = colorDetection.getPosition();
        telemetry.addLine(String.valueOf(avg));
        telemetry.addLine(colorDetection.toString());
        telemetry.update();

    }
}
