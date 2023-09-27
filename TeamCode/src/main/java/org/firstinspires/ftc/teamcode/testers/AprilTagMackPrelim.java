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

@TeleOp(name = "April tag")
public class AprilTagMackPrelim extends LinearOpMode {

    private static final int ID = 0;
    final double DISTANCE = 12.0;

    final double DRIVE_GAIN = 0.02;
    final double STRAFE_GAIN = 0.015;
    final double TURN_GAIN = 0.01;

    private DcMotor fl;
    private DcMotor bl;
    private DcMotor fr;
    private DcMotor br;

    private VisionPortal vp;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection wantedTag = null;

    @Override
    public void runOpMode() {
        boolean isDetected = false;

        double drive = 0;
        double strafe = 0;
        double turn = 0;

        initialize();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            for (AprilTagDetection d : currentDetections) {
                if (d.metadata != null && d.id == ID) {
                    isDetected = true;
                    wantedTag = d;
                }
            }

            if (gamepad1.cross && isDetected) {
                double rangeError = wantedTag.ftcPose.range - DISTANCE;
                double bearingError = wantedTag.ftcPose.bearing;
                double yawError = wantedTag.ftcPose.yaw;

                drive = Range.clip(rangeError * DRIVE_GAIN, -1, 1);
                turn = Range.clip(bearingError * TURN_GAIN, -1, 1);
                strafe = Range.clip(yawError * STRAFE_GAIN, -1, 1);
            } else {
                double ls_x = gamepad1.left_stick_x;
                double ls_y = gamepad1.left_stick_y;
                double rs_x = -gamepad1.right_stick_x;

                fl.setPower((ls_x - ls_y + 0.8*rs_x) / 2.8);
                fr.setPower((ls_x + ls_y - 0.8*rs_x) / 2.8);
                bl.setPower((ls_x + ls_y + 0.8*rs_x) / 2.8);
                br.setPower((ls_x - ls_y - 0.8*rs_x) / 2.8);
            }

            move(drive, strafe, turn);
            sleep(10);
        }
    }

    private void initialize() {
        aprilTag = new AprilTagProcessor.Builder().build();
        vp = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        fl = hardwareMap.get(DcMotor.class, "fl");
        bl = hardwareMap.get(DcMotor.class, "bl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        br = hardwareMap.get(DcMotor.class, "br");

        setManualExposure(6, 250); // Guess

        // xx.setDirection(DcMotor.Direction.REVERSE);
    }

    private void move(double drive, double strafe, double turn) {
        double flV = (drive - strafe - turn) / 3;
        double frV = (drive + strafe + turn) / 3;
        double blV = (drive + strafe - turn) / 3;
        double brV = (drive - strafe + turn) / 3;

        fl.setPower(flV);
        fr.setPower(frV);
        bl.setPower(blV);
        br.setPower(brV);
    }

    private void setManualExposure(int e, int g) {
        if (vp == null) {
            return;
        }

        while (!isStopRequested() && (vp.getCameraState() != VisionPortal.CameraState.STREAMING)) {
            sleep(20);
        }

        if (!isStopRequested()) {
            ExposureControl exposureControl = vp.getCameraControl(ExposureControl.class);

            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }

            exposureControl.setExposure((long)e, TimeUnit.MILLISECONDS);
            sleep(20);

            GainControl gainControl = vp.getCameraControl(GainControl.class);
            gainControl.setGain(g);
            sleep(20);
        }
    }
}
