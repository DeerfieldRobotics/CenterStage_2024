package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.utils.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.utils.DrivetrainKotlin;
import org.firstinspires.ftc.teamcode.utils.WhichPath;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@TeleOp(name = "auto meet 1")
public class Auto_v1 extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    SimpleMotorFeedforward feedForward;

    WhichPath whichPath = new WhichPath();

    int path = -2;

    //static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    //size of small tags, size of large tags = 0.124, all units are meters
    double tagsize = 0.0508;
    double tagOffset = 0.1524; //Distance between center of tags

    int numFramesWithoutDetection = 0;

    final float DECIMATION_HIGH = 3;
    final float DECIMATION_LOW = 2;
    final float THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f;
    final int THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4;

    private PIDController xController, zController, yawController;

    //all these values need to be tuned
    public static double pX = 0, iX = 0, dX = 0, pZ = 0, iZ = 0, dZ = 0, pYaw = 0, iYaw = 0, dYaw = 0;
    public static double kS = 0, kV = 0, kA = 0;
    //targetY is left right relative to april tag and targetZ is back forward relative to april tag
    //need to calibrate
    public double targetX = 0, targetZ = 0.2, targetYaw = 0;
    public double[] currentX = {0, 0, 0}, currentZ = {0, 0, 0}, currentYaw = {0, 0, 0}; //left tag is 0, center tag is 1, right tag is 2
    private DrivetrainKotlin drivetrain;

    @Override
    public void runOpMode()
    {
        initialize();

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        waitForStart();

        telemetry.setMsTransmissionInterval(50);

        while (opModeIsActive())
        {




        }
    }

    public void initialize(){
        xController = new PIDController(pX, iX, dX);
        zController = new PIDController(pZ, iZ, dZ);
        yawController = new PIDController(pYaw, iYaw, dYaw);
        feedForward = new SimpleMotorFeedforward(kS, kV, kA);
        xController.setSetPoint(targetX);
        zController.setSetPoint(targetZ);
        yawController.setSetPoint((targetYaw));
//        xController.setTolerance(tagOffset/3.0);
//        zController.setTolerance(0.05);
//        yawController.setTolerance(2);



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.setPipeline(whichPath);
//        camera = OpenCvCameraFactory.getInstance()
//                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId); //Cell phone camera
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        /*
            Which path to take???????
         */

        path = whichPath.getPath();

        telemetry.addLine("Width: " +whichPath.getDims()[0] + " Height: " + whichPath.getDims()[1]);
        telemetry.addLine("0: " +whichPath.getCnts()[0] + " 1: " + whichPath.getCnts()[1] + " 2: " + whichPath.getCnts()[2]);

        telemetry.update();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drivetrain = new DrivetrainKotlin(hardwareMap);
    }



}
