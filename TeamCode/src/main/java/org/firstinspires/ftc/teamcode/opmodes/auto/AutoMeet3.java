package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleop;
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
        BLUE_CLOSE,
        BLUE_FAR,
        RED_FAR,
        RED_CLOSE
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
    private double initTangent;
    private double purpleTangent;
    private double centerBackup = 0;
    private Pose2d preLowerWhitePose;
    private Pose2d whitePixelStackPose;
    private Pose2d postLowerWhitePose;
    private double preLowerWhiteTangent;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        slide = new Slide(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap, slide);

        outtake.setGateClosed(true);
        outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.INSIDE);
        outtake.update();

        intake.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
        TrajectorySequence prePath = drive.trajectorySequenceBuilder(initPose)
                .setTangent(initTangent)
                .addTemporalMarker(()->{
                    intake.setServoPosition(Intake.IntakePositions.DRIVE);
                    intake.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    intake.update();
                })
                .splineToSplineHeading(purplePose, purpleTangent)
                .back(centerBackup)
                .addTemporalMarker(()->{
                    outtakePurple();
                })
                .waitSeconds(0.2)
                .setTangent(0)
                .addTemporalMarker(()->{
                    raiseSlide();
                })
                .splineToLinearHeading(backdropPose, Math.toRadians(0))
                .addTemporalMarker(()->{
                    outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.OUTSIDE);
                    outtake.update();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(()->{
                    slide.setTargetPosition(-290);
                    slide.update();
                })
                .waitSeconds(0.2)
                .back(3.5)
                .waitSeconds(0.3)
                .addTemporalMarker(()->{
                    outtake.setGateClosed(false);
                    outtake.update();
                })
                .waitSeconds(0.4)
                .addTemporalMarker(()-> {
                    outtake.setGateClosed(true);
                    outtake.update();
                    slide.setTargetPosition(-1400);
                    slide.update();
                })
                .forward(10)
                .waitSeconds(0.2)
                .addTemporalMarker(()->{
                    outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.INSIDE);
                    outtake.update();
                })
                .waitSeconds(0.7)
                // TODO: BRING SLIDE DOWN, RAISE INTAKE
                .setTangent(preLowerWhiteTangent)
                .addTemporalMarker(()->{
                    slide.setBottomOutProcedure(true);
                    intake.setServoPosition(Intake.IntakePositions.FIVE);
                    intake.update();
                })
                .splineToSplineHeading(preLowerWhitePose, Math.toRadians(180))
                .addTemporalMarker(()->{
                    outtake.setGateClosed(false);
                    outtake.update();
                })
                .splineToSplineHeading(whitePixelStackPose, Math.toRadians(180))
                .addTemporalMarker(()->{
                    intake.setMotorTargetPosition(800);
                    intake.setMotorPower(0.8);
                    intake.update();
                })
                .waitSeconds(.6)
                .back(4)
                .addTemporalMarker(()-> {
                    intake.setServoPosition(Intake.IntakePositions.TRANSFER);
                    intake.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    intake.update();
                    outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.TRANSFER);
                    outtake.update();
                })
                .waitSeconds(0.4)
                .addTemporalMarker(()->{
                    intake.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    intake.setMotorPower(1);
                    intake.update();
                })
                .waitSeconds(0.7)
                .addTemporalMarker(()->{
                    intake.setMotorPower(0);
                    intake.setServoPosition(Intake.IntakePositions.DRIVE);
                    intake.update();
                    outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.INSIDE);
                    outtake.update();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()->{
                    outtake.setGateClosed(true);
                    outtake.update();
                })
                .setTangent(0)
                .splineToSplineHeading(postLowerWhitePose, 0)

                .splineToSplineHeading(backdropPose, Math.toRadians(45))
                .addTemporalMarker(()->{
                    outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.INSIDE);
                    outtake.update();
                    raiseSlide();
                })
                .waitSeconds(0.7)
                .addTemporalMarker(()->{
                    outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.OUTSIDE);
                    outtake.update();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(()->{
                    slide.setTargetPosition(-700);
                    slide.update();
                })
                .waitSeconds(0.2)
                .back(6)
                .waitSeconds(0.6)
                // DROP
                .addTemporalMarker(()->{
                    outtake.setGateClosed(false);
                    outtake.update();
                })
                .waitSeconds(0.4)
                .addTemporalMarker(()-> {
                    outtake.setGateClosed(true);
                    outtake.update();
                    slide.setTargetPosition(-1400);
                    slide.update();
                })
                .forward(7)
                .addTemporalMarker(()->{
                    outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.INSIDE);
                    outtake.update();
                })
                .waitSeconds(0.8)
                .addTemporalMarker(()->{
                    slide.setBottomOutProcedure(true);
                    intake.setServoPosition(Intake.IntakePositions.FIVE);
                    intake.update();
                })
                .build();

        drive.followTrajectorySequenceAsync(prePath);
    }

    private void outtakePurple() {
        intake.setMotorTargetPosition(250);
        intake.setMotorPower(0.5);
        intake.update();
    }

    private void raiseSlide() {
        slide.setTargetPosition(-1400);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1);
        slide.update();
    }

    @Override
    public void loop() {
        drive.update();
        outtake.update();
        intake.update();
        slide.update();
        telemetry.update();
    }

    private void initColorDetection() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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
            case BLUE_CLOSE:
                initPose = new Pose2d(16, 63, Math.toRadians(270));
                purplePose = new Pose2d(-63, -48, Math.toRadians(0));
                purpleTangent = Math.toRadians(210);
                initTangent = Math.toRadians(300);
                preLowerWhitePose = new Pose2d(24,11, Math.toRadians(180));
                whitePixelStackPose = new Pose2d(-57,-6.5, Math.toRadians(180));//This had better be the same every time TY
                postLowerWhitePose = new Pose2d(28, 10, Math.toRadians(180));
                preLowerWhiteTangent = 225;
                switch (purplePixelPath) {
                    case LEFT:
                        purplePose = new Pose2d(32.5,30, Math.toRadians(180));
                        backdropPose = new Pose2d(-63, -48, Math.toRadians(0)); // TODO see below
                        break;
                    case CENTER:
                        purplePose = new Pose2d(23,24.2, Math.toRadians(180));
                        backdropPose = new Pose2d(-63, -48, Math.toRadians(0)); // TODO see below
                        centerBackup = 0;
                        break;
                    case RIGHT:
                        purplePose = new Pose2d(12.5,30, Math.toRadians(180));
                        backdropPose = new Pose2d(-63, -48, Math.toRadians(0)); // TODO see below
                        break;
                }
                break;
            case RED_CLOSE:
                initPose = new Pose2d(11, -63, Math.toRadians(90));
                purplePose = new Pose2d(-63, -48, Math.toRadians(0));
                purpleTangent = Math.toRadians(150);
                initTangent = Math.toRadians(90);
                preLowerWhitePose = new Pose2d(24,-10, Math.toRadians(180));
                whitePixelStackPose = new Pose2d(-57,16, Math.toRadians(180));
                postLowerWhitePose = new Pose2d(28, -10, Math.toRadians(180));
                preLowerWhiteTangent = 135;
                switch (purplePixelPath) {
                    case LEFT:
                        purplePose = new Pose2d(32.5,30, Math.toRadians(180));
                        backdropPose = new Pose2d(54, -48, Math.toRadians(0)); // *this is below* TODO adjust for april tag estimate to get tag in frame
                        centerBackup = 3.5; // FIX THIS POOP
                        break;
                    case CENTER:
                        purplePose = new Pose2d(23,24.2, Math.toRadians(180));
                        backdropPose = new Pose2d(54, -48, Math.toRadians(0)); // TODO adjust for april tag estimate to get tag in frame
                        centerBackup = 3.5; // FIX THIS POOP
                        break;
                    case RIGHT:
                        purplePose = new Pose2d(12.5,30, Math.toRadians(180));
                        backdropPose = new Pose2d(54, -48, Math.toRadians(0)); // TODO adjust for april tag estimate to get tag in frame
                        centerBackup = 3.5-8; // FIX THIS POOP
                        break;
                }
                break;
            case BLUE_FAR:
                // TODO: Write auto
                break;
            case RED_FAR:
                // TODO: Write auto
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
                startPosition = START_POSITION.BLUE_CLOSE;
                MainTeleop.alliance = AprilTagAlignment.Alliance.BLUE;
                break;
            }
            if (gamepad1.dpad_down) {
                startPosition = START_POSITION.BLUE_FAR;
                MainTeleop.alliance = AprilTagAlignment.Alliance.BLUE;
                break;
            }
            if (gamepad1.dpad_left) {
                startPosition = START_POSITION.RED_FAR;
                MainTeleop.alliance = AprilTagAlignment.Alliance.RED;
                break;
            }
            if (gamepad1.dpad_right) {
                startPosition = START_POSITION.RED_CLOSE;
                MainTeleop.alliance = AprilTagAlignment.Alliance.RED;
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }
}
