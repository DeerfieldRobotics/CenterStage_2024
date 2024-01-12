package org.firstinspires.ftc.teamcode.testers;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.hardware.Drivetrain;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl;

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
    private final AprilTagLibrary aprilTagLibrary = new AprilTagLibrary.Builder().addTag(6, "RedRight", 6.5, DistanceUnit.INCH).build();
    private AprilTagProcessorImpl processor = new AprilTagProcessorImpl(0, 0, 0, 0, DistanceUnit.INCH, AngleUnit.DEGREES, aprilTagLibrary, false, false, false, false, AprilTagProcessor.TagFamily.TAG_36h11, 3); // Used for managing the AprilTag detection process.
    private AprilTagDetection detection = null; // Used to hold the data for a detected AprilTag

    //driving values
    private double speedMult;
    private double forwardMult = 1;
    private double turnMult = .75;
    private double strafeMult = 1.48;

//    DrivetrainKotlin drivetrain;

    SampleMecanumDrive drivetrain;
    @Override public void runOpMode()
    {
        drivetrain = new SampleMecanumDrive(hardwareMap);
        boolean targetFound = false;
        double drive = 0;
        double strafe = 0;
        double turn = 0;

        initAprilTag();

//        drivetrain = new DrivetrainKotlin(hardwareMap);
//        drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        setManualExposure(6, 250);

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

            drivetrain.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drivetrain.update();

            sleep(10);
        }
    }

    private void driveNormal() {
        speedMult = .7+0.3 * gamepad1.right_trigger-0.3*gamepad1.left_trigger;

        gamepad1.rumble(gamepad1.left_trigger>0.5?(gamepad1.left_trigger-0.5)/.4:0.0,gamepad1.right_trigger>0.4?(gamepad1.right_trigger-0.4)/0.8:0.0,50);

        double forward = gamepad1.left_stick_y * forwardMult * speedMult;
        double turn = -gamepad1.right_stick_x * turnMult * speedMult;
        double strafe = -gamepad1.left_stick_x * strafeMult * speedMult;

//        drivetrain.move(forward, strafe, turn);
    }

    private void initAprilTag() {
        // Create the vision portal by using a builder.
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setCameraResolution(new Size(800, 600))
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
