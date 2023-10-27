package org.firstinspires.ftc.teamcode.testers;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.ColorDetectionPipelineJames;
import org.firstinspires.ftc.teamcode.utils.ColorDetectionPipelineWorking;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@Autonomous(name = "ColorDetectionTestWorking")
public class ColorDetectionTestWorking extends LinearOpMode {
    //OpenCvCamera frontCamera;
    private OpenCvCamera frontCamera;

    private int path, red;

    private ColorDetectionPipelineJames detector = new ColorDetectionPipelineJames();


    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //frontCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "frontWeb"), cameraMonitorViewId);
        frontCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        frontCamera.setPipeline(detector);
        frontCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                frontCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        while (opModeInInit()) {
            String p = detector.getPosition();

            if (p == "RIGHT") path = 1;
            else if (p == "LEFT") path = -1;
            else path = 0;

            String c = detector.getColor();

            if (c == "RED") red = 1;
            else red = 0;

            telemetry.addData("Position", detector.getPosition());
            telemetry.addData("Color", detector.getColor());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {

        }
    }
}