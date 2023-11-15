package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.testers.PIDF;
import org.firstinspires.ftc.teamcode.utils.WhiteDetectionPipeline;
import org.firstinspires.ftc.teamcode.utils.IntakeKotlin;
import org.firstinspires.ftc.teamcode.utils.SlideKotlin;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "WhiteTEst")
public class WhiteTest extends OpMode {
    private final WhiteDetectionPipeline colorDetection = new WhiteDetectionPipeline();
    private OpenCvInternalCamera frontCamera;
    //private OpenCvCamera frontCamera;
    private double centerx = 0;
    private double mult = 0.0;

    private PIDF pidf;
    private SampleMecanumDrive drive;
    private IntakeKotlin intake;
    private SlideKotlin slide;
    private TrajectorySequence path;
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

        avg = colorDetection.getAvg();


//        if(purplePixelPath.equals(ColorDetectionPipeline.StartingPosition.CENTER)) {
//            mult = 0.0;
//            centerx = 5;
//        }
//        else if (purplePixelPath.equals(ColorDetectionPipeline.StartingPosition.LEFT)) mult = 1.0;
//        else mult = -1.0;

        telemetry.addLine(String.valueOf(avg));

        double[] s = colorDetection.getSelected();

        telemetry.addLine(String.valueOf(s[0]));
        telemetry.addLine(String.valueOf(s[1]));
        telemetry.addLine(String.valueOf(s[2]));
        telemetry.addLine(String.valueOf(s[3]));

        telemetry.addData("max white", colorDetection.getMaxWhite());
        telemetry.addLine(colorDetection.toString());

        telemetry.update();
    }

    @Override
    public void start() {
//        drive.setPoseEstimate(new Pose2d(8.25,-63, Math.toRadians(90)));
//        path = drive.trajectorySequenceBuilder(new Pose2d(8.25,-63, Math.toRadians(90)))
//                .addTemporalMarker(3.5, ()->{
//                    slide.setTargetPosition(-1000);
//                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slide.setPower(-1);
//                    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//                    frontCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//                    frontCamera.setPipeline(colorDetection);
//                    frontCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//                        @Override
//                        public void onOpened() {
//                            frontCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//                        }
//                        @Override
//                        public void onError(int errorCode) {}
//                    });
//                })
//                        .build();
//        drive.followTrajectorySequenceAsync(path);
    }
    @Override
    public void loop() {
//        avg = cp.getAvg();
//        telemetry.addLine(""+ slide.getPosition()[0]);
//        telemetry.addLine("AVG "+cp.getAvg());
//        telemetry.addLine("WHITE VALUES: "+cp.getWhiteVals());
//
//        telemetry.update();
    }
}
