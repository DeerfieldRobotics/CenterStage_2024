package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utils.detection.AprilTagAlignment;
import org.firstinspires.ftc.teamcode.utils.detection.ColorDetectionPipeline;
import org.firstinspires.ftc.teamcode.utils.detection.WhiteDetectionPipeline;
import org.firstinspires.ftc.teamcode.utils.hardware.Intake;
import org.firstinspires.ftc.teamcode.utils.hardware.Outtake;
import org.firstinspires.ftc.teamcode.utils.hardware.Slide;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Auto Meet 3", preselectTeleOp = "MainTeleop")
public class AutoMeet3 extends OpMode {
    //Define and declare Robot Starting Locations
    private enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    private static START_POSITION startPosition;

    private AprilTagAlignment aprilTagAlignment;
    private final ColorDetectionPipeline colorDetectionPipeline = new ColorDetectionPipeline();
    private final WhiteDetectionPipeline whiteDetectionPipeline = new WhiteDetectionPipeline();
    private ColorDetectionPipeline.StartingPosition purplePixelPath = ColorDetectionPipeline.StartingPosition.CENTER;
    private SampleMecanumDrive drive;
    private Intake intake;
    private Outtake outtake;
    private Slide slide;
    private OpenCvCamera frontCamera;
    private WebcamName backCamera;
    private Pose2d initPose;
    private Pose2d purplePose;
    private Pose2d backdropPose;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        slide = new Slide(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap, slide);

        outtake.setGateClosed(true);
        outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.INSIDE);
        outtake.update();

        initColorDetection();
    }
    @Override
    public void init_loop() {
        detectPurplePath();
        buildAuto();
    }
    @Override
    public void start() {
        drive.setPoseEstimate(initPose);
        TrajectorySequence path = drive.trajectorySequenceBuilder(initPose)
                .lineToLinearHeading(purplePose)
                .build();

    }

    @Override
    public void loop() {
        drive.update();
        outtake.update();
        intake.update();
        slide.update();
        telemetry.update();
    }

    private void initColorDetection() {int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        frontCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        frontCamera.setPipeline(colorDetectionPipeline);
        frontCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                frontCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {}
        });
    }
    private void detectPurplePath() {
        purplePixelPath = colorDetectionPipeline.getPosition();
        telemetry.addLine(colorDetectionPipeline.toString());
        telemetry.addData("Purple Pixel Path: ", purplePixelPath);
        telemetry.update();
    }
    public void buildAuto() { //TODO
        switch (startPosition) {
            case BLUE_LEFT:
                initPose = new Pose2d(16, 63, Math.toRadians(270));
                purplePose = new Pose2d(-63, -48, Math.toRadians(0));
                backdropPose = new Pose2d(-63, -48, Math.toRadians(0));
                break;
            case BLUE_RIGHT:
                initPose = new Pose2d(-63, -48, Math.toRadians(0));
                purplePose = new Pose2d(-63, -48, Math.toRadians(0));
                backdropPose = new Pose2d(-63, -48, Math.toRadians(0));
                break;
            case RED_LEFT:
                initPose = new Pose2d(-63, -48, Math.toRadians(0));
                purplePose = new Pose2d(-63, -48, Math.toRadians(0));
                backdropPose = new Pose2d(-63, -48, Math.toRadians(0));
                break;
            case RED_RIGHT:
                initPose = new Pose2d(-63, -48, Math.toRadians(0));
                purplePose = new Pose2d(-63, -48, Math.toRadians(0));
                backdropPose = new Pose2d(-63, -48, Math.toRadians(0));
                break;
        }
    }

    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while (getRuntime() < 30) {
            telemetry.addLine("15118 Auto Initialized");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Select Starting Position using DPAD Keys on gamepad 1:", "");
            telemetry.addData("    Blue Left   ", "(↑)");
            telemetry.addData("    Blue Right ", "(↓)");
            telemetry.addData("    Red Left    ", "(←)");
            telemetry.addData("    Red Right  ", "(→)");
            if (gamepad1.dpad_up) {
                startPosition = START_POSITION.BLUE_LEFT;
                break;
            }
            if (gamepad1.dpad_down) {
                startPosition = START_POSITION.BLUE_RIGHT;
                break;
            }
            if (gamepad1.dpad_left) {
                startPosition = START_POSITION.RED_LEFT;
                break;
            }
            if (gamepad1.dpad_right) {
                startPosition = START_POSITION.RED_RIGHT;
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }
}
