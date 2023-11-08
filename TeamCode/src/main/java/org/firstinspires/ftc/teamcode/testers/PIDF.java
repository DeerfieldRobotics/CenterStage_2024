package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.utils.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.utils.DrivetrainKotlin;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@TeleOp(name = "PIDFTest", group = "TeleOp")
@Disabled
public class PIDF extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    SimpleMotorFeedforward feedForward;

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
    public static double targetX = 0, targetZ = 0.2, targetYaw = 0;
    public double[] currentX = {0, 0, 0}, currentZ = {0, 0, 0}, currentYaw = {0, 0, 0}; //left tag is 0, center tag is 1, right tag is 2
    private DrivetrainKotlin drivetrain;

    @Override
    public void runOpMode()
    {
        initialize();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        camera = OpenCvCameraFactory.getInstance()
//                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId); //Cell phone camera
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

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
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            // If there's been a new frame...
            if(detections != null)
            {
                telemetry.addData("FPS", camera.getFps());
                telemetry.addData("Overhead ms", camera.getOverheadTimeMs());
                telemetry.addData("Pipeline ms", camera.getPipelineTimeMs());

                // If we don't see any tags
                if(detections.size() == 0)
                {
                    numFramesWithoutDetection++;

                    // If we haven't seen a tag for a few frames, lower the decimation
                    // so we can hopefully pick one up if we're e.g. far back
                    if(numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW);
                    }
                }
                // We do see tags!
                else
                {
                    numFramesWithoutDetection = 0;

                    // If the target is within 1 meter, turn on high decimation to
                    // increase the frame rate
                    if(detections.get(0).pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS)
                    {
                        aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH);
                    }

                    currentX = new double[]{0, 0, 0}; //reset all current values before updating
                    currentZ = new double[]{0, 0, 0};
                    currentYaw = new double[]{0, 0, 0};
                    xController.setSetPoint(targetX);
                    zController.setSetPoint(targetZ);
                    yawController.setSetPoint((targetYaw));

                    for (AprilTagDetection detection : detections)
                    {
                        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
//
//                        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//                        telemetry.addLine(String.format("Translation X: %.2f meters", detection.pose.x));
//                        telemetry.addLine(String.format("Translation Y: %.2f meters", detection.pose.y));
//                        telemetry.addLine(String.format("Translation Z: %.2f meters", detection.pose.z));
//                        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", rot.firstAngle));
//                        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", rot.secondAngle));
//                        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", rot.thirdAngle));

                        if(detection.id == 1||detection.id==4) {
                            currentX[0] = detection.pose.x;
                            currentZ[0] = detection.pose.z;
                            currentYaw[0] = rot.firstAngle;
                        }
                        else if (detection.id == 2 || detection.id == 5) {
                            currentX[1] = detection.pose.x;
                            currentZ[1] = detection.pose.z;
                            currentYaw[1] = rot.firstAngle;
                        }
                        else if (detection.id == 3 || detection.id == 6) {
                            currentX[2] = detection.pose.x;
                            currentZ[2] = detection.pose.z;
                            currentYaw[2] = rot.firstAngle;
                        }
                    }
                    double oneFourDistance = Math.abs(currentX[0]) + Math.abs(currentZ[0]);
                    double twoFiveDistance = Math.abs(currentX[1]) + Math.abs(currentZ[1]);
                    double threeSixDistance = Math.abs(currentX[2] + Math.abs(currentZ[2]));
                    double distMin = Double.MAX_VALUE;
                    double arr[] = {oneFourDistance, twoFiveDistance, threeSixDistance};
                    for (double d : arr)
                    {
                        distMin = (d == 0) ? distMin : Math.min(distMin, d);
                    }
                    yawController.setPID(pYaw, iYaw,dYaw);
                    xController.setPID(pX, iX,dX);
                    zController.setPID(pZ, iZ,dZ);

                    double calcX = 0, calcZ = 0, calcYaw = 0;
//                    if(distMin == oneFourDistance){
//                        calcX = currentX[0];
//                        calcZ = currentZ[0];
//                        calcYaw = currentYaw[0];
//                    }
//                    else if(distMin == twoFiveDistance){
//                        calcX = currentX[1];
//                        calcZ = currentZ[1];
//                        calcYaw = currentYaw[1];
//                    }
//                    else {
//                        calcX = currentX[2];
//                        calcZ = currentZ[2];
//                        calcYaw = currentYaw[2];
//                    }
                        calcX = currentX[1];
                        calcZ = currentZ[1];
                        calcYaw = currentYaw[1];
                        telemetry.addData("calcX", calcX);
                        telemetry.addData("calcZ", calcZ);
                        telemetry.addData("calcYaw", calcYaw);

                    //telemetry.addLine("got past min distance");
//                    while(!zController.atSetPoint() && !xController.atSetPoint()) {
                        double outputX = xController.calculate(calcX);
                        double outputZ = zController.calculate(calcZ);
                        double outputYaw = yawController.calculate(calcYaw);
                        telemetry.addData("outputX", outputX);
                        telemetry.addData("outputZ", outputZ);
                        telemetry.addData("outputYaw", outputYaw);
                        telemetry.addData("targetX", targetX);
                        telemetry.addData("targetZ", targetZ);
                        telemetry.addData("targetYaw", targetYaw);
//                        feedForward.calculate(1, 1);
//                        if(!yawController.atSetPoint()) {
                            telemetry.addLine("adjusting Yaw");
                            drivetrain.move(0, 0, outputYaw);
//                        }
//                        if(!xController.atSetPoint()) {
                            telemetry.addLine("adjusting X");
                            drivetrain.move(0, outputX, 0);
//                        }
//                        if(!zController.atSetPoint()){
                            telemetry.addLine("adjusting Z");
                            drivetrain.move(outputZ, 0, 0);
//                        }
//                    }
                }



                telemetry.update();
            }

            sleep(20);
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
        zController.setTolerance(0.005);
        yawController.setTolerance(0.05);



        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drivetrain = new DrivetrainKotlin(hardwareMap);
    }



}
