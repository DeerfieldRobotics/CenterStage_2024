package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Drive to april tag")
public class DriveToTag extends LinearOpMode {
    final double DISTANCE = 12.0; // Inches away from tag

    // Gains: x% power at n inches/degrees error => (x / n) where x is a decimal.
    final double SPEED_GAIN = 0.02; // 50% at 25 inches (.5/25)
    final double STRAFE_GAIN = 0.015; // 25% at 25 deg (.25/25)
    final double TURN_GAIN = 0.01; // 25% at 25 deg (.25/25)

    final double MAX_SPEED = 0.5;
    final double MAX_STRAFE = 0.5;
    final double MAX_TURN = 0.3;

    private DcMotor flMotor;
    private DcMotor frMotor;
    private DcMotor blMotor;
    private DcMotor brMotor;

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

        flMotor = hardwareMap.get(DcMotor.class, "fl");
        frMotor = hardwareMap.get(DcMotor.class, "fr");
        blMotor = hardwareMap.get(DcMotor.class, "bl");
        brMotor = hardwareMap.get(DcMotor.class, "br");

        brMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.REVERSE);
        flMotor.setDirection(DcMotor.Direction.REVERSE);

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
                    telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData(">","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Target", "ID %d (%s)", detection.id, detection.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", detection.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", detection.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", detection.ftcPose.yaw);
            } else {
                telemetry.addData(">","Drive using joysticks to find valid target\n");
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (gamepad1.left_bumper && targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (detection.ftcPose.range - DISTANCE);
                double  headingError    = detection.ftcPose.bearing;
                double  yawError        = detection.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_SPEED, MAX_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_TURN, MAX_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_STRAFE, MAX_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            } else {

                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
                strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
                turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            sleep(10);
        }
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        flMotor.setPower(leftFrontPower);
        frMotor.setPower(rightFrontPower);
        blMotor.setPower(leftBackPower);
        brMotor.setPower(rightBackPower);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        processor = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(processor)
                    .build();
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

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
