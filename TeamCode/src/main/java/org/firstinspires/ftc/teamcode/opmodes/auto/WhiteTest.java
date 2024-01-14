package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utils.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.detection.WhiteDetectionPipeline;
import org.firstinspires.ftc.teamcode.utils.hardware.Intake;
import org.firstinspires.ftc.teamcode.utils.hardware.Slide;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "WhiteTest")
public class WhiteTest extends OpMode {
    private final WhiteDetectionPipeline colorDetection = new WhiteDetectionPipeline();
//    private OpenCvInternalCamera frontCamera;
    private OpenCvCamera frontCamera;
    private double centerx = 0;
    private double mult = 0.0;

//    private SampleMecanumDrive drive;
    private Drivetrain drive;
    private Intake intake;
    private Slide slide;
    private TrajectorySequence path;
    private double stackOffset = 0;

//    private ColorDetectionPipeline.StartingPosition purplePixelPath;
//    private ColorDetectionPipeline cp = new ColorDetectionPipeline("WHITE");

    private double avg = -1;

    @Override
    public void init() {
//        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        frontCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        frontCamera = OpenCvCameraFactory.getInstance()
//                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);



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

        drive = new Drivetrain(hardwareMap);
    }

    @Override
    public void init_loop() {
//        purplePixelPath = colorDetection.getPosition();

        avg = colorDetection.getAvg();

        telemetry.addLine(String.valueOf(avg));

        telemetry.addLine(colorDetection.toString());

        telemetry.update();
    }

    @Override
    public void start() {
//        drive.
    }
    @Override
    public void loop() {
        while(Math.abs(avg-3.5)>= 1) {
            drive.strafe((int)((3.5-avg)*10));
        }
    }
}
