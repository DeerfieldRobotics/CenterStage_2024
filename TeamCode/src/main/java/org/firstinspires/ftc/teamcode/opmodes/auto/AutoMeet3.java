package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.CogchampDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utils.Other.Datalogger;

import org.firstinspires.ftc.teamcode.utils.detection.AllianceHelper;
import org.firstinspires.ftc.teamcode.utils.detection.AprilTagAlignmentProcessor;
import org.firstinspires.ftc.teamcode.utils.detection.ColorDetectionProcessor;
import org.firstinspires.ftc.teamcode.utils.hardware.Intake;
import org.firstinspires.ftc.teamcode.utils.hardware.Outtake;
import org.firstinspires.ftc.teamcode.utils.hardware.Slide;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@Autonomous(name = "Auto Meet 3", preselectTeleOp = "Main Teleop", group = "a")
public class AutoMeet3 extends LinearOpMode {
    //Define and declare Robot Starting Locations
    private enum START_POSITION {
        //IN RELATION TO BACKBOARD
        BLUE_CLOSE,
        BLUE_FAR,
        RED_FAR,
        RED_CLOSE,
        // ????
        UNKNOWN
    }

    private static START_POSITION startPosition = START_POSITION.UNKNOWN; //WHERE WE ARE ON THE FIELD/ RED CLOSE ETC

    private Datalog datalog; //TELEMETRY
    private ColorDetectionProcessor colorDetectionProcessor;
    private AprilTagAlignmentProcessor aprilTagProcessorBack;
    private AprilTagAlignmentProcessor aprilTagProcessorFront;
    private VisionPortal frontCameraPortal;
    private VisionPortal backCameraPortal;
    private ColorDetectionProcessor.StartingPosition purplePixelPath = ColorDetectionProcessor.StartingPosition.CENTER;
    private CogchampDrive drive;
    private Intake intake;
    private Outtake outtake;
    private Slide slide;
    private VoltageSensor battery;

    // TRAJECTORIES
    private TrajectorySequence spikeThenBackboard; // SPIKE THEN GO TO BACKBOARD
    private TrajectorySequence pathBackboardToWhite; //BACKBOARD TO STACK
    private TrajectorySequence pathWhiteToBackboard;// STACK BACK TO BACKBOARD
    private TrajectorySequence pathBackboardToPark; //PARK AFTER SCORING STACK

    //POSITIONS & TANGENTS
    private Pose2d initPose; //INITIAL POSITION
    private Pose2d purplePose; //SPIKE POSITION
    private Pose2d aprilTagPose; // POSITION AFTER APRIL TAGS
    private Pose2d aprilTagPose2;
    private Pose2d backboardPose; // POSITION WHERE BACKBOARD SHOULD BE, STARTING POSITION TO STACK
    private double backboardApriltagX; // X VALUE OF LOCATION RELATIVE TO BACKBOARD IN INCHES
    private double secondBackboardApriltagX; // X VALUE OF LOCATION RELATIVE TO BACKBOARD IN INCHES
    private double initTangent; // INITIAL TANGENT 60 degrees
    private double purpleTangent; // TANGENT TO SPIKE
    private double centerBackup = 0; //BACKUP FROM SPIKE //TODO: WHY DO WE NEED THIS??????
    private Pose2d beforeStackPose; //before we go to the stack,
    private Pose2d preWhitePose;
    private Pose2d whitePixelStackPose;
    private Pose2d postLowerWhitePose; //pose after
    private double preLowerWhiteTangent;


    @Override
    public void runOpMode() throws InterruptedException {
        selectStartingPosition(); //FIND FIELD POSITION & COLOR
        initialize(); //INITIALIZE CLASSES

        while(!isStarted() && !isStopRequested()) {
            initLoop();// LEFT/CENTER/RIGHT???
        }

        waitForStart();

        startAuto(); // GO!!!!!!

        while (opModeIsActive() && !isStopRequested()) {
            autoLoop(); //update classes
        }
    }

    public void initialize() {
        drive = new CogchampDrive(hardwareMap);
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

        initPortals();
    }

    public void initLoop() {
        detectPurplePath();
        telemetry.addData("Selected Auto: ", startPosition.toString());
        telemetry.update();
    }

    public void startAuto() {
        frontCameraPortal.stopStreaming();
        backCameraPortal.resumeLiveView();
        telemetry.clear();
        buildAuto(); //INITIALIZE POSITIONS
        datalog.opModeStatus.set("RUNNING");
        drive.setPoseEstimate(initPose);
        spikeThenBackboard = drive.trajectorySequenceBuilder(initPose)
                .setTangent(initTangent)
                .addTemporalMarker(()->{ intake.setServoPosition(Intake.IntakePositions.DRIVE); })
                .splineToSplineHeading(purplePose, purpleTangent)
                .back(centerBackup) //BACKUP FROM SPIKE
                .addTemporalMarker(this::outtakePurple) // SPIKE AND SCORE
                .waitSeconds(0.2)
                .setTangent(0)
                // GO TO BACKBOARD
                .splineToLinearHeading(aprilTagPose, Math.toRadians(0))
                .addTemporalMarker(this::outtake)
                .addTemporalMarker(()->{
                    aprilTagProcessorBack.setTargetX(backboardApriltagX);})
                .addTemporalMarker(()->{
                    alignToApriltag();
                    drive.setPoseEstimate(backboardPose);
                    drive.followTrajectorySequenceAsync(pathBackboardToWhite);
                })
                .build();

        pathBackboardToWhite = drive.trajectorySequenceBuilder(backboardPose) //GO TO STACK
                .waitSeconds(0.2)
                .setTangent(0)
                .back(5)
                .waitSeconds(0.2)
                .addTemporalMarker(this::drop)
                .waitSeconds(0.3)
                .addTemporalMarker(()->{ setSlideHeight(-1200); })
                .waitSeconds(0.4)
                .addTemporalMarker(this::outtakeIn)
                .forward(10)
                .setTangent(preLowerWhiteTangent)
                .addTemporalMarker(()->{ intake.setServoPosition(Intake.IntakePositions.FOUR); intake.update(); })
                .splineToConstantHeading(new Vector2d(beforeStackPose.getX(), beforeStackPose.getY()), Math.toRadians(180))
                .addTemporalMarker(() -> {
//                    colorPortal.resumeStreaming(); //probably do this earlier
//                    colorPortal.setProcessorEnabled(whiteDetectionProcessor, true);
//                    alignToWhite();
                    drive.followTrajectorySequenceAsync(pathWhiteToBackboard);
                })
                .build();

        pathWhiteToBackboard = drive.trajectorySequenceBuilder(preWhitePose) // TODO: WTF IS THIS?????????????
                .splineToSplineHeading(whitePixelStackPose, Math.toRadians(180))
                .forward(2)
                .addTemporalMarker(()->{
                    intake.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    intake.intakePower(1.0);
                    intake.update();
                })
                .addTemporalMarker(this::outtakeTransfer)
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                    intake.setBoosterServoPower(-1.0);
                    intake.update();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()->{
                    intake.setBoosterServoPower(0.0);
                    intake.update();
                })
                .back(4)
                .addTemporalMarker(this::transfer)
                .waitSeconds(0.7)
//                .addTemporalMarker(this::outtakeIn)
                .setTangent(0)
                .splineToSplineHeading(postLowerWhitePose, 0)
                .splineToSplineHeading(aprilTagPose2, Math.toRadians(45))
                .addTemporalMarker(()->{
                    setSlideHeight(-1400);
                })
                .addTemporalMarker(this::outtake)
                .addTemporalMarker(()->{
                    aprilTagProcessorBack.setTargetX(secondBackboardApriltagX);})
                .addTemporalMarker(()->{
                    alignToApriltag();
                    backCameraPortal.close();
                    frontCameraPortal.close();
                    drive.setPoseEstimate(backboardPose);
                    drive.followTrajectorySequenceAsync(pathBackboardToPark);
                })
                .build();
        pathBackboardToPark = drive.trajectorySequenceBuilder(backboardPose)
                .back(4.5)
                .waitSeconds(0.6)
                .addTemporalMarker(this::drop)
                .addTemporalMarker(() -> {
                    setSlideHeight(-1400);
                })
                .waitSeconds(0.4)
                .addTemporalMarker(this::outtakeIn)
                .forward(7)
                .build();

        drive.followTrajectorySequenceAsync(spikeThenBackboard);
    }

    private void alignToApriltag() {
        double currentTime = getRuntime();
        while(!isStopRequested()) {
            aprilTagProcessorBack.update();
            aprilTagProcessorBack.alignRobot(drive);

            telemetry.addData("x error","%5.1f inches", aprilTagProcessorBack.getXError());
            telemetry.addData("y error","%5.1f inches", aprilTagProcessorBack.getYError());
            telemetry.addData("heading error","%3.0f degrees", aprilTagProcessorBack.getHeadingError());
            telemetry.addData("drivetrain power", drive.getPoseEstimate());
            telemetry.update();
            intake.update();
            slide.update();
            outtake.update();

            if(getRuntime()-currentTime > 2) break;
        }
    }

    public void autoLoop() {
        drive.update();
        outtake.update();
        intake.update();
        slide.update();
    }

    private void initPortals() {
        CameraName backCamera = hardwareMap.get(WebcamName.class, "Webcam 1");
        CameraName frontCamera = hardwareMap.get(WebcamName.class, "Webcam 2");

        aprilTagProcessorBack = new AprilTagAlignmentProcessor(AprilTagAlignmentProcessor.CameraType.BACK, 12.0, 0.0, 0.0, AllianceHelper.alliance); // Used for managing the april tag detection process.
        aprilTagProcessorFront = new AprilTagAlignmentProcessor(AprilTagAlignmentProcessor.CameraType.FRONT, 12.0, 0.0, 0.0, AllianceHelper.alliance); // Used for managing the april tag detection process.
        colorDetectionProcessor = new ColorDetectionProcessor(AllianceHelper.alliance); // Used for managing the color detection process.

        List<Integer> portalList = JavaUtil.makeIntegerList(VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL));
        int frontPortalId = (Integer) JavaUtil.inListGet(portalList, JavaUtil.AtMode.FROM_START, 0, false);
        int backPortalId = (Integer) JavaUtil.inListGet(portalList, JavaUtil.AtMode.FROM_START, 1, false);

        frontCameraPortal = new VisionPortal.Builder()
                .setCamera(frontCamera)
                .setCameraResolution(new android.util.Size(320, 240))
                .addProcessors(colorDetectionProcessor, aprilTagProcessorFront)
                .setLiveViewContainerId(frontPortalId)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        backCameraPortal = new VisionPortal.Builder()
                .setCamera(backCamera)
                .setCameraResolution(new android.util.Size(1280, 720))
                .addProcessor(aprilTagProcessorBack)
                .setLiveViewContainerId(backPortalId)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

        backCameraPortal.stopLiveView();
    }
    private void detectPurplePath() {
        purplePixelPath = colorDetectionProcessor.getPosition();
        telemetry.addData("Purple Pixel Path: ", purplePixelPath.toString());
    }

    public void buildAuto() {
        switch (startPosition) {
            case BLUE_CLOSE:
                initPose = new Pose2d(18, 63, Math.toRadians(270));
                purplePose = new Pose2d(11, 25, Math.toRadians(180));
                purpleTangent = Math.toRadians(180);
                initTangent = Math.toRadians(300);
                beforeStackPose = new Pose2d(24,14, Math.toRadians(180));
                preWhitePose = new Pose2d(24, 14, Math.toRadians(180));
                postLowerWhitePose = new Pose2d(28, 14, Math.toRadians(180));
                preLowerWhiteTangent = 225;
                switch (purplePixelPath) {
                    case RIGHT:
                        purplePose = new Pose2d(12,32, Math.toRadians(180));
                        aprilTagPose = new Pose2d(52, 31, Math.toRadians(180)); // *this is below* TODO adjust for april tag estimate to get tag in frame
                        backboardPose = new Pose2d(52, 32, Math.toRadians(180)); // TODO see below
                        aprilTagPose2 = new Pose2d(48, 42, Math.toRadians(180));
                        backboardApriltagX = -7;
                        secondBackboardApriltagX = 7;
                        centerBackup = 4; // FIX THIS POOP
                        whitePixelStackPose = new Pose2d(-53,17.5, Math.toRadians(180));
                        break;
                    case CENTER:
                        purplePose = new Pose2d(23,24.2, Math.toRadians(180));
                        aprilTagPose = new Pose2d(52, 39, Math.toRadians(180)); // TODO adjust for april tag estimate to get tag in frame
                        backboardPose = new Pose2d(52, 39, Math.toRadians(180)); // TODO see below
                        aprilTagPose2 = new Pose2d(48, 33, Math.toRadians(180));
                        backboardApriltagX = 0;
                        secondBackboardApriltagX = 7;
                        centerBackup = 3.5; // FIX THIS POOP
                        whitePixelStackPose = new Pose2d(-54,18.5, Math.toRadians(180));
                        break;
                    case LEFT:
                        purplePose = new Pose2d(36,32, Math.toRadians(180));
                        aprilTagPose = new Pose2d(52, 42, Math.toRadians(180)); // TODO adjust for april tag estimate to get tag in frame
                        backboardPose = new Pose2d(52, 41, Math.toRadians(180)); // TODO see below
                        aprilTagPose2 = new Pose2d(48, 33, Math.toRadians(180));
                        backboardApriltagX = 7;
                        secondBackboardApriltagX = -7;
                        centerBackup = 1.5; // FIX THIS POOP
                        whitePixelStackPose = new Pose2d(-53,13, Math.toRadians(180));
                        break;
                }
                break;
            case RED_CLOSE:
                initPose = new Pose2d(11, -63, Math.toRadians(90));
                purplePose = new Pose2d(11, -25, Math.toRadians(180));
                purpleTangent = Math.toRadians(180);
                initTangent = Math.toRadians(60);
                beforeStackPose = new Pose2d(24,-14, Math.toRadians(180));
                whitePixelStackPose = new Pose2d(-57,-12, Math.toRadians(180));
                preWhitePose = new Pose2d(24, -14, Math.toRadians(180));
                postLowerWhitePose = new Pose2d(28, -14, Math.toRadians(180));
                preLowerWhiteTangent = 135;
                switch (purplePixelPath) {
                    case LEFT:
                        purplePose = new Pose2d(11,-32, Math.toRadians(180));
                        aprilTagPose = new Pose2d(52, -30, Math.toRadians(180)); // *this is below* TODO adjust for april tag estimate to get tag in frame
                        backboardPose = new Pose2d(52, -30, Math.toRadians(180)); // TODO see below
                        aprilTagPose2 = new Pose2d(48, -39, Math.toRadians(180));
                        backboardApriltagX = 7;
                        secondBackboardApriltagX = -7;
                        centerBackup = 5.5; // FIX THIS POOP
                        whitePixelStackPose = new Pose2d(-52,-11, Math.toRadians(180));
                        break;
                    case CENTER:
                        purplePose = new Pose2d(20,-24.2, Math.toRadians(180));
                        aprilTagPose = new Pose2d(52, -39, Math.toRadians(180)); // TODO adjust for april tag estimate to get tag in frame
                        backboardPose = new Pose2d(52, -39, Math.toRadians(180)); // TODO see below
                        aprilTagPose2 = new Pose2d(48, -30, Math.toRadians(180));
                        backboardApriltagX = 0;
                        secondBackboardApriltagX = 7;
                        centerBackup = 5.5; // FIX THIS POOP
                        whitePixelStackPose = new Pose2d(-52,-10, Math.toRadians(180));
                        break;
                    case RIGHT:
                        purplePose = new Pose2d(33,-32, Math.toRadians(180));
                        aprilTagPose = new Pose2d(52, -41, Math.toRadians(180)); // TODO adjust for april tag estimate to get tag in frame
                        backboardPose = new Pose2d(52, -45, Math.toRadians(180)); // TODO see below
                        aprilTagPose2 = new Pose2d(48, -30, Math.toRadians(180));
                        backboardApriltagX = -7;
                        secondBackboardApriltagX = 7;
                        centerBackup = 5.5; // FIX THIS POOP
                        whitePixelStackPose = new Pose2d(-52,-7.5, Math.toRadians(180));
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
        intake.update();
    }

    private void outtakePurple() {
        intake.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.update();
        intake.setMotorTargetPosition(-400);
        intake.update();
        intake.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.update();
        intake.setMotorPower(0.4);
        intake.update();
    }

    private void outtake() {
        outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.OUTSIDE);
        if(slide.getAvgPosition() >= -900) {
            setSlideHeight(-900);
        }
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

    private void outtakeTransfer() {
        outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.TRANSFER);
        outtake.update();
    }

    private void transfer() {
        intake.transfer();
        intake.update();
    }

    //Method to select starting position using dpad on gamepad
    public void selectStartingPosition() {
        //******select start pose*****
        while (!isStopRequested()) {
            telemetry.addLine("Auto Meet 3 Initialized");
            telemetry.addData("---------------------------------------", "");
            telemetry.addLine("Select Starting Position using DPAD Keys");
            telemetry.addData("    Blue Left   ", "(^)");
            telemetry.addData("    Blue Right ", "(v)");
            telemetry.addData("    Red Left    ", "(<)");
            telemetry.addData("    Red Right  ", "(>)");

            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                startPosition = START_POSITION.BLUE_CLOSE;
                AllianceHelper.alliance = AllianceHelper.Alliance.BLUE;
                break;
            }
            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                startPosition = START_POSITION.BLUE_FAR;
                AllianceHelper.alliance = AllianceHelper.Alliance.BLUE;
                break;
            }
            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                startPosition = START_POSITION.RED_FAR;
                AllianceHelper.alliance = AllianceHelper.Alliance.RED;
                break;
            }
            if (gamepad1.dpad_right || gamepad2.dpad_right) {
                startPosition = START_POSITION.RED_CLOSE;
                AllianceHelper.alliance = AllianceHelper.Alliance.RED;
                break;
            }
            telemetry.update();
        }
        telemetry.clear();
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
