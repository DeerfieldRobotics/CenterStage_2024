package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opmodes.auto.PoseHelper;
import org.firstinspires.ftc.teamcode.roadrunner.drive.CogchampDrive;
import org.firstinspires.ftc.teamcode.utils.detection.AprilTagAlignmentProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@TeleOp(name = "April Tag Tester")
public class AprilTagTester extends LinearOpMode {
    private CogchampDrive drivetrain;
    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new CogchampDrive(hardwareMap);

        CameraName backCamera = hardwareMap.get(WebcamName.class, "Webcam 1");
        CameraName frontCamera = hardwareMap.get(WebcamName.class, "Webcam 2");

        AprilTagAlignmentProcessor aprilTagAlignmentProcessorBack = new AprilTagAlignmentProcessor(AprilTagAlignmentProcessor.CameraType.BACK, PoseHelper.backboardCenterRed);
        AprilTagAlignmentProcessor aprilTagAlignmentProcessorFront = new AprilTagAlignmentProcessor(AprilTagAlignmentProcessor.CameraType.FRONT, PoseHelper.apriltagStackRed);

        List<Integer> portalList = JavaUtil.makeIntegerList(VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL));
        int frontPortalId = (Integer) JavaUtil.inListGet(portalList, JavaUtil.AtMode.FROM_START, 0, false);
        int backPortalId = (Integer) JavaUtil.inListGet(portalList, JavaUtil.AtMode.FROM_START, 1, false);

        VisionPortal frontCameraPortal = new VisionPortal.Builder()
                .setCamera(frontCamera)
                .setCameraResolution(new android.util.Size(1280, 720))
                .addProcessors(aprilTagAlignmentProcessorFront)
                .setLiveViewContainerId(frontPortalId)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        VisionPortal backCameraPortal = new VisionPortal.Builder()
                .setCamera(backCamera)
                .setCameraResolution(new android.util.Size(1280, 720))
                .addProcessor(aprilTagAlignmentProcessorBack)
                .setLiveViewContainerId(backPortalId)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        waitForStart();

        drivetrain.setPoseEstimate(PoseHelper.initCloseRed);

        while(opModeIsActive() && !isStopRequested()) {
            aprilTagAlignmentProcessorBack.update();
            aprilTagAlignmentProcessorFront.update();

            if (gamepad1.left_bumper) {
                aprilTagAlignmentProcessorBack.alignRobot(drivetrain);
                drivetrain.setPoseEstimate(aprilTagAlignmentProcessorBack.getPoseEstimate());
            } else if (gamepad1.right_bumper) {
                aprilTagAlignmentProcessorFront.alignRobot(drivetrain);
                drivetrain.setPoseEstimate(aprilTagAlignmentProcessorBack.getPoseEstimate());
            }
            else
                driveNormal();

            telemetry.addData("Back Camera Pose", "x: %3.2f in, y: %3.2f in, heading %3.2f°",aprilTagAlignmentProcessorBack.getPoseEstimate().getX(),aprilTagAlignmentProcessorBack.getPoseEstimate().getY(),aprilTagAlignmentProcessorBack.getPoseEstimate().getHeading());
            telemetry.addData("Back Camera Error", "x: %3.2f in, y: %3.2f in, heading %3.2f°", aprilTagAlignmentProcessorBack.getPoseError().getX(), aprilTagAlignmentProcessorBack.getPoseError().getY(), aprilTagAlignmentProcessorBack.getPoseError().getHeading());
            telemetry.addData("Front Camera Pose", "x: %3.2f in, y: %3.2f in, heading %3.2f°",aprilTagAlignmentProcessorFront.getPoseEstimate().getX(),aprilTagAlignmentProcessorFront.getPoseEstimate().getY(),aprilTagAlignmentProcessorFront.getPoseEstimate().getHeading());
            telemetry.addData("Front Camera Error", "x: %3.2f in, y: %3.2f in, heading %3.2f°",aprilTagAlignmentProcessorFront.getXError(),aprilTagAlignmentProcessorFront.getYError(),aprilTagAlignmentProcessorFront.getHeadingError());
            telemetry.update();
        }
    }

    private void driveNormal() {
        //driving values
        double speedMult = .7 + 0.3 * gamepad1.right_trigger - 0.3 * gamepad1.left_trigger;

        gamepad1.rumble(gamepad1.left_trigger > 0.5 ? (gamepad1.left_trigger - 0.5) / .4 : 0.0, gamepad1.right_trigger > 0.4 ? (gamepad1.right_trigger - 0.4) / 0.8 : 0.0, 50);

        double forwardMult = 1;
        double turnMult = .75;
        double strafeMult = 1;

        double forward = -gamepad1.left_stick_y * forwardMult * speedMult;
        double turn = -gamepad1.right_stick_x * turnMult * speedMult;
        double strafe = -gamepad1.left_stick_x * strafeMult * speedMult;

        drivetrain.setWeightedDrivePower(
                new Pose2d(
                        forward,
                        strafe,
                        turn
                )
        );
    }
}
