package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.sun.tools.javac.Main;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opmodes.teleop.MainTeleop;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utils.Other.Datalogger;
import org.firstinspires.ftc.teamcode.utils.detection.AprilTagAlignment;
import org.firstinspires.ftc.teamcode.utils.detection.ColorDetectionPipeline;
import org.firstinspires.ftc.teamcode.utils.detection.WhiteDetectionPipeline;
import org.firstinspires.ftc.teamcode.utils.hardware.Drivetrain;
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

    private Datalog datalog;
    private AprilTagAlignment aprilTagAlignment;
    private final ColorDetectionPipeline colorDetectionPipeline = new ColorDetectionPipeline();
    private final WhiteDetectionPipeline whiteDetectionPipeline = new WhiteDetectionPipeline();
    private ColorDetectionPipeline.StartingPosition purplePixelPath = ColorDetectionPipeline.StartingPosition.CENTER;
    private SampleMecanumDrive drive;
    private Drivetrain drivetrain;
    private Intake intake;
    private Outtake outtake;
    private Slide slide;
    private OpenCvCamera frontCamera;
    private WebcamName backCamera;
    private VoltageSensor battery;

    private TrajectorySequence pathInitToBackboard;
    private TrajectorySequence pathBackboardToWhite;
    private TrajectorySequence pathWhiteToBackboard;
    private TrajectorySequence pathBackboardToPark;

    private Pose2d initPose;
    private Pose2d purplePose;
    private Pose2d aprilTagPose;
    private Pose2d backboardPose;
    private double initTangent;
    private double purpleTangent;
    private double centerBackup = 0;
    private Pose2d whiteDetectionPose;
    private Pose2d preWhitePose;
    private Pose2d whitePixelStackPose;
    private Pose2d postLowerWhitePose;
    private double preLowerWhiteTangent;


    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        slide = new Slide(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap, slide);

        battery = hardwareMap.voltageSensor.get("Control Hub");

        datalog = new Datalog("AutoDatalogger");

        datalog.opModeStatus.set("INIT");
        datalog.battery.set(battery.getVoltage());
        datalog.writeLine();

        outtake.setGateClosed(true);
        outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.INSIDE);
        outtake.update();

        intake.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        initAprilTagDetection();

        initColorDetection();
        selectStartingPosition();
    }

    @Override
    public void init_loop() {
        detectPurplePath();
        buildAuto();
    }

    @Override
    public void start() {
        datalog.opModeStatus.set("RUNNING");
        drive.setPoseEstimate(initPose);
        pathInitToBackboard = drive.trajectorySequenceBuilder(initPose)
                .setTangent(initTangent)
                .addTemporalMarker(()->{ intake.setServoPosition(Intake.IntakePositions.DRIVE); })
                .splineToSplineHeading(purplePose, purpleTangent)
                .back(centerBackup)
                .addTemporalMarker(this::outtakePurple)
                .waitSeconds(0.2)
                .setTangent(0)
                .splineToLinearHeading(aprilTagPose, Math.toRadians(0))
                .addTemporalMarker(()->{
                    double currentTime = getRuntime();
                    while(getRuntime() - currentTime < 3) { //TODO find the number of seconds, optimal would be 2 sd over mean time to reach apriltag
                        //TODO might need to disable samplemecanum drive idk tho
                        aprilTagAlignment.alignRobotToBackboard(MainTeleop.alliance);

                        telemetry.addData("x error","%5.1f inches", aprilTagAlignment.getXError());
                        telemetry.addData("y error","%5.1f inches", aprilTagAlignment.getYError());
                        telemetry.addData("heading error","%3.0f degrees", aprilTagAlignment.getHeadingError());
                        telemetry.addData("drivetrain power", drive.getPoseEstimate());
                        telemetry.update();

                        datalog.xError.set(aprilTagAlignment.getXError());
                        datalog.yError.set(aprilTagAlignment.getYError());
                        datalog.headingError.set(aprilTagAlignment.getHeadingError());
                        datalog.writeLine();

                        if(aprilTagAlignment.getTargetFound()) break;
                    }
                    drive.followTrajectorySequenceAsync(pathBackboardToWhite);
                })
                .build();
        pathBackboardToWhite = drive.trajectorySequenceBuilder(backboardPose)
                .addTemporalMarker(this::outtake)
                .waitSeconds(0.5)
                .addTemporalMarker(()->{ setSlideHeight(-240); })
                .waitSeconds(0.2)
                .back(3.5)
                .waitSeconds(0.3)
                .addTemporalMarker(this::drop)
                .waitSeconds(0.4)
                .addTemporalMarker(this::outtakeIn)
                .forward(10)
                .waitSeconds(0.7)
                .setTangent(preLowerWhiteTangent)
                .addTemporalMarker(()->{ intake.setServoPosition(Intake.IntakePositions.FIVE); })
                .splineToSplineHeading(whiteDetectionPose, Math.toRadians(180))
                .addTemporalMarker(() -> {
                    //TODO add white detection
                    drive.followTrajectorySequenceAsync(pathWhiteToBackboard);
                })
                .build();
        pathWhiteToBackboard = drive.trajectorySequenceBuilder(preWhitePose)
                .addTemporalMarker(this::intake)
                .splineToSplineHeading(whitePixelStackPose, Math.toRadians(180))
                .addTemporalMarker(this::transfer)
                .waitSeconds(.6)
                .back(4)
                .waitSeconds(0.7)
                .addTemporalMarker(this::outtakeIn)
                .setTangent(0)
                .splineToSplineHeading(postLowerWhitePose, 0)
                .splineToSplineHeading(aprilTagPose, Math.toRadians(45))
                .addTemporalMarker(this::outtake)
                .waitSeconds(0.5)
                .addTemporalMarker(()->{ setSlideHeight(-400); })
                .waitSeconds(0.2)
                .back(6)
                .waitSeconds(0.6)
                .addTemporalMarker(this::drop)
                .waitSeconds(0.4)
                .addTemporalMarker(this::outtakeIn)
                .forward(7)
                .build();

        drive.followTrajectorySequenceAsync(pathInitToBackboard);
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
    private void initAprilTagDetection() {
        backCamera = hardwareMap.get(WebcamName.class, "Webcam 1");

        aprilTagAlignment = new AprilTagAlignment(backCamera, drivetrain, 0.0, 12.0, 0.0,
                (new PIDController(0.0174, 0.0, 0.0)), //x PID controller
                (new PIDController(0.0174, 0.0, 0.0)), //y PID controller
                (new PIDController(0.0174, 0.0, 0.0))); //heading PID controller
    }
    public void buildAuto() { //TODO
        switch (startPosition) {
            case BLUE_CLOSE:
                initPose = new Pose2d(16, 63, Math.toRadians(270));
                purplePose = new Pose2d(-63, -48, Math.toRadians(0));
                purpleTangent = Math.toRadians(210);
                initTangent = Math.toRadians(300);
                whiteDetectionPose = new Pose2d(24,11, Math.toRadians(180));
                whitePixelStackPose = new Pose2d(-57,-6.5, Math.toRadians(180));//This had better be the same every time TY
                postLowerWhitePose = new Pose2d(28, 10, Math.toRadians(180));
                preLowerWhiteTangent = 225;
                switch (purplePixelPath) {
                    case LEFT:
                        purplePose = new Pose2d(32.5,30, Math.toRadians(180));
                        aprilTagPose = new Pose2d(-63, -48, Math.toRadians(0)); // TODO see below
                        break;
                    case CENTER:
                        purplePose = new Pose2d(23,24.2, Math.toRadians(180));
                        aprilTagPose = new Pose2d(-63, -48, Math.toRadians(0)); // TODO see below
                        centerBackup = 0;
                        break;
                    case RIGHT:
                        purplePose = new Pose2d(12.5,30, Math.toRadians(180));
                        aprilTagPose = new Pose2d(-63, -48, Math.toRadians(0)); // TODO see below
                        break;
                }
                break;
            case RED_CLOSE:
                initPose = new Pose2d(11, -63, Math.toRadians(90));
                purplePose = new Pose2d(-63, -48, Math.toRadians(0));
                purpleTangent = Math.toRadians(150);
                initTangent = Math.toRadians(90);
                whiteDetectionPose = new Pose2d(24,-10, Math.toRadians(180));
                whitePixelStackPose = new Pose2d(-57,16, Math.toRadians(180));
                postLowerWhitePose = new Pose2d(28, -10, Math.toRadians(180));
                preLowerWhiteTangent = 135;
                switch (purplePixelPath) {
                    case LEFT:
                        purplePose = new Pose2d(32.5,30, Math.toRadians(180));
                        aprilTagPose = new Pose2d(54, -48, Math.toRadians(0)); // *this is below* TODO adjust for april tag estimate to get tag in frame
                        centerBackup = 3.5; // FIX THIS POOP
                        break;
                    case CENTER:
                        purplePose = new Pose2d(23,24.2, Math.toRadians(180));
                        aprilTagPose = new Pose2d(54, -48, Math.toRadians(0)); // TODO adjust for april tag estimate to get tag in frame
                        centerBackup = 3.5; // FIX THIS POOP
                        break;
                    case RIGHT:
                        purplePose = new Pose2d(12.5,30, Math.toRadians(180));
                        aprilTagPose = new Pose2d(54, -48, Math.toRadians(0)); // TODO adjust for april tag estimate to get tag in frame
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
    private void intake() {
        intake.intake(.75);
    }
    private void outtakePurple() {
        intake.setMotorTargetPosition(250);
        intake.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setMotorPower(0.5);
        intake.update();
    }
    private void outtake() {
        outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.OUTSIDE);
        setSlideHeight(-1200);
    }
    private void outtakeIn() {
        if(outtake.getOuttakePosition() == Outtake.OuttakePositions.OUTSIDE)
            setSlideHeight(-1200);
        outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.INSIDE);
    }
    private void drop() {
        outtake.setGateClosed(false);
    }
    private void setSlideHeight(int height) {
        slide.setTargetPosition(height);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(1.0);
    }
    private void transfer() {
        intake.transfer();
    }

    //Method to select starting position using dpad on gamepad
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
    public static class Datalog
    {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField opModeStatus = new Datalogger.GenericField("OpModeStatus");
        public Datalogger.GenericField loopCounter  = new Datalogger.GenericField("Loop Counter");
        public Datalogger.GenericField xError       = new Datalogger.GenericField("X Error");
        public Datalogger.GenericField yError       = new Datalogger.GenericField("Y Error");
        public Datalogger.GenericField headingError = new Datalogger.GenericField("Heading Error");
        public Datalogger.GenericField battery      = new Datalogger.GenericField("Battery");

        public Datalog(String name)
        {
            // Build the underlying datalog object
            datalogger = new Datalogger.Builder()

                    // Pass through the filename
                    .setFilename(name)

                    // Request an automatic timestamp field
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                    // Tell it about the fields we care to log.
                    // Note that order *IS* important here! The order in which we list
                    // the fields is the order in which they will appear in the log.
                    .setFields(
                            opModeStatus,
                            loopCounter,
                            xError,
                            yError,
                            headingError,
                            battery
                    )
                    .build();
        }

        // Tell the datalogger to gather the values of the fields
        // and write a new line in the log.
        public void writeLine()
        {
            datalogger.writeLine();
        }
    }
}
