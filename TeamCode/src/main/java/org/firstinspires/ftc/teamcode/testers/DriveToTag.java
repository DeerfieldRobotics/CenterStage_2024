package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.utils.DrivetrainKotlin;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Drive to april tag")
public class DriveToTag extends LinearOpMode {
    final double DISTANCE = 12.0; // Inches away from tag

    final double MAX_SPEED = 0.5;
    final double MAX_STRAFE = 0.5;
    final double MAX_TURN = 0.3;

    PIDController speedController   = new PIDController(0.02, 0, 0);
    PIDController headingController = new PIDController(0.01, 0, 0);
    PIDController strafeController  = new PIDController(0.015, 0, 0);

    private static final int TAG_ID = 6; // -1 for any tag
    private VisionPortal visionPortal;
    private AprilTagProcessor processor; // Used for managing the AprilTag detection process.
    private AprilTagDetection detection = null; // Used to hold the data for a detected AprilTag

    @Override public void runOpMode()
    {
        boolean targetFound = false;
        double drive = 0;
        double strafe = 0;
        double turn = 0;

        initAprilTag();

        DrivetrainKotlin drivetrain = new DrivetrainKotlin(hardwareMap);
        drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setManualExposure(6, 250);

        telemetry.update();
        waitForStart();

        while (opModeIsActive())
        {
            targetFound = false;
            detection = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = processor.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null) &&
                        ((TAG_ID < 0) || (detection.id == TAG_ID))  ){
                    targetFound = true;
                    this.detection = detection;
                    break;  // don't look any further.
                } else {
                    telemetry.addData("Unknown Target", "Tag ID %d", detection.id);
                }
            }

            if (targetFound) {
                telemetry.addData(">","Detection found, hold LB\n");
                telemetry.addData("Target", "ID %d (%s)", detection.id, detection.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", detection.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", detection.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", detection.ftcPose.yaw);
            } else {
                telemetry.addData(">","Drive\n");
            }

            if (gamepad1.left_bumper && targetFound) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError   = detection.ftcPose.range - DISTANCE;
                double headingError = detection.ftcPose.bearing;
                double yawError     = detection.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = speedController.calculate(rangeError);
                turn   = headingController.calculate(headingError);
                strafe = strafeController.calculate(yawError);
            } else {
                drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
                strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
                turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
           }
            telemetry.update();

            drivetrain.move(drive, strafe, turn);
            sleep(10);
        }
    }

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        processor = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(processor)
                    .build();
    }

    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}
