package org.firstinspires.ftc.teamcode.testers;


import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.ColorDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.utils.Drivetrain;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "ColorDetection", group = "OpenCvTesting")
public class ColorDetectionTest extends LinearOpMode{
    private OpenCvInternalCamera camera;
    private ColorDetectionPipeline colorDetection;
    private Drivetrain drivetrain;
    public ColorDetectionTest(OpenCvInternalCamera c, Drivetrain d){
        camera = c;
        drivetrain = d;
    }


    //need to implement this too
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while(opModeIsActive())
        {
//            telemetry.addData("Frame Count", camera.getFrameCount());
//            telemetry.addData("FPS", camera.getFps());
//            telemetry.addData("Total frame time ms", camera.getTotalFrameTimeMs());
//            telemetry.addData("Pipeline time ms", camera.getPipelineTimeMs());
//            telemetry.addData("Overhead time ms", camera.getOverheadTimeMs());
//            telemetry.addData("Theoretical max FPS", camera.getCurrentPipelineMaxFps());
            String pos = colorDetection.getPropPosition();
            telemetry.addData("Starting Position", pos);
            int[][] totalRedBluePixels = colorDetection.getTotalRedBlue();
            telemetry.addData("Total Left Red ", totalRedBluePixels[0][0]);
            telemetry.addData("Total Center Red", totalRedBluePixels[0][1]);
            telemetry.addData("Total Right Red", totalRedBluePixels[0][2]);
            telemetry.addData("Total Left Blue ", totalRedBluePixels[1][0]);
            telemetry.addData("Total Center Blue", totalRedBluePixels[1][1]);
            telemetry.addData("Total Right Blue", totalRedBluePixels[1][2]);
            telemetry.update();
        }
    }

    public void initialize(){
        colorDetection = new ColorDetectionPipeline();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        camera.setPipeline(colorDetection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //potentially use width: 640 and height: 480
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }
}
