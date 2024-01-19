package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.roadrunner.drive.CogchampDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utils.Other.Datalogger;
import com.arcrobotics.ftclib.controller.PIDController;
import org.firstinspires.ftc.teamcode.utils.detection.AllianceHelper;
import org.firstinspires.ftc.teamcode.utils.detection.AprilTagAlignmentAuto;
import org.firstinspires.ftc.teamcode.utils.detection.ColorDetectionProcessor;
import org.firstinspires.ftc.teamcode.utils.detection.WhiteDetectionPipeline;
import org.firstinspires.ftc.teamcode.utils.detection.WhiteDetectionProcessor;
import org.firstinspires.ftc.teamcode.utils.hardware.Intake;
import org.firstinspires.ftc.teamcode.utils.hardware.Outtake;
import org.firstinspires.ftc.teamcode.utils.hardware.Slide;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl;

import java.util.List;

@Autonomous(name = "Auto Meet 3", preselectTeleOp = "MainTeleop", group = "a")
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
    private AprilTagAlignmentAuto aprilTagAlignment; //APRIL TAG DETECTION
    private ColorDetectionProcessor colorDetectionProcessor;
    private WhiteDetectionProcessor whiteDetectionProcessor;
    private AprilTagProcessorImpl aprilTagProcessor;
    private VisionPortal colorPortal;
    private VisionPortal aprilTagPortal;
    private boolean runAprilTag = true;
    private ColorDetectionProcessor.StartingPosition purplePixelPath = ColorDetectionProcessor.StartingPosition.CENTER;
    private CogchampDrive drive;
    private Intake intake;
    private Outtake outtake;
    private Slide slide;
    private VoltageSensor battery;

    //PID values
    public static double xP = 0.06;
    public static double xI = 0.03;
    public static double xD = 0.0006;
    public static double yP = 0.04;
    public static double yI = 0.04;
    public static double yD = 0.0009;
    public static double headingP = 0.02;
    public static double headingI = 0.035;
    public static double headingD = 0.001;
    private PIDController xPID;
    private PIDController yPID;
    private PIDController headingPID;
    private boolean positionFound = false;

    // TRAJECTORIES
    private TrajectorySequence spikeThenBackboard; // SPIKE THEN GO TO BACKBOARD
    private TrajectorySequence pathBackboardToWhite; //BACKBOARD TO STACK
    private TrajectorySequence pathWhiteToBackboard;// STACK BACK TO BACKBOARD
    private TrajectorySequence pathBackboardToPark; //PARK AFTER SCORING STACK

    //POSITIONS & TANGENTS
    private Pose2d initPose; //INITIAL POSITION
    private Pose2d purplePose; //SPIKE POSITION
    private Pose2d aprilTagPose; // POSITION AFTER APRIL TAGS
    private Pose2d backboardPose; // POSITION WHERE BACKBOARD SHOULD BE, STARTING POSITION TO STACK
    private double backboardApriltagX; // X VALUE OF LOCATION RELATIVE TO BACKBOARD IN INCHES
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

        xPID = new PIDController(xP, xI, xD);
        yPID = new PIDController(yP, yI, yD);
        headingPID = new PIDController(headingP, headingI, headingD);

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
        colorPortal.stopStreaming();
        aprilTagPortal.resumeStreaming();
        telemetry.clear();
        buildAuto(); //INITIALIZE POSITIONS
        datalog.opModeStatus.set("RUNNING");
        drive.setPoseEstimate(initPose);
        if (startPosition == START_POSITION.BLUE_CLOSE || startPosition == START_POSITION.RED_CLOSE) { //CLOSE TO BACKBOARD
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
                    .addTemporalMarker(()->{aprilTagAlignment.setTargetX(backboardApriltagX);})
                    .addTemporalMarker(()->{
                        alignToApriltag();
                        drive.followTrajectorySequenceAsync(pathBackboardToWhite);
                    })
                    .build();
        }
        else {
            spikeThenBackboard = drive.trajectorySequenceBuilder(initPose) // NOT CLOSE TO BACKBOARD
                    .setTangent(initTangent)
                    .addTemporalMarker(()->{ intake.setServoPosition(Intake.IntakePositions.DRIVE); })
                    .splineToSplineHeading(purplePose, purpleTangent)
                    .back(centerBackup)
                    .addTemporalMarker(this::outtakePurple)
                    .waitSeconds(0.2)
                    .setTangent(0)
                    .splineToLinearHeading(aprilTagPose, Math.toRadians(0))
                    .addTemporalMarker(()->{
                        alignToApriltag();
                        drive.followTrajectorySequenceAsync(pathBackboardToWhite);
                    })
                    .build();
        }

        pathBackboardToWhite = drive.trajectorySequenceBuilder(backboardPose) //GO TO STACK
                .waitSeconds(0.2)
                .addTemporalMarker(()->runAprilTag = false)
                .back(5)
                .waitSeconds(0.5)
                .addTemporalMarker(this::drop)
                .waitSeconds(0.4)
                .addTemporalMarker(this::outtakeIn)
                .forward(10)
                .waitSeconds(0.7)
                .setTangent(preLowerWhiteTangent)
                .addTemporalMarker(()->{ intake.setServoPosition(Intake.IntakePositions.FIVE); })
                .splineToSplineHeading(beforeStackPose, Math.toRadians(180))
                .addTemporalMarker(() -> {
                    //TODO add white detection
                    colorPortal.resumeStreaming(); //probably do this earlier
                    colorPortal.setProcessorEnabled(whiteDetectionProcessor, true);
                    whiteDetectionProcessor.alignRobot(drive);
                    drive.followTrajectorySequenceAsync(pathWhiteToBackboard);
                })
                .build();

        pathWhiteToBackboard = drive.trajectorySequenceBuilder(preWhitePose) // TODO: WTF IS THIS?????????????
                .addTemporalMarker(this::intake)
                .splineToSplineHeading(whitePixelStackPose, Math.toRadians(180))
                .addTemporalMarker(this::transfer)
                .waitSeconds(.6)
                .back(4)
                .waitSeconds(0.7)
                .addTemporalMarker(this::outtakeIn)
                .setTangent(0)
                .splineToSplineHeading(postLowerWhitePose, 0)
                .splineToConstantHeading(new Vector2d(aprilTagPose.getX(), aprilTagPose.getY()), Math.toRadians(45))
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

        drive.followTrajectorySequenceAsync(spikeThenBackboard);
    }

    private void alignToApriltag() {
        double currentTime = getRuntime();
        while(runAprilTag && !isStopRequested()) {
            aprilTagAlignment.update();
            aprilTagAlignment.alignRobotToBackboard(drive);

            telemetry.addData("x error","%5.1f inches", aprilTagAlignment.getXError());
            telemetry.addData("y error","%5.1f inches", aprilTagAlignment.getYError());
            telemetry.addData("heading error","%3.0f degrees", aprilTagAlignment.getHeadingError());
            telemetry.addData("drivetrain power", drive.getPoseEstimate());
            telemetry.update();
            slide.update();
            outtake.update();

            if(aprilTagAlignment.robotAligned() && getRuntime()-currentTime > 3) break;
        }
        runAprilTag = true;
        drive.followTrajectorySequenceAsync(pathBackboardToWhite);
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

        AprilTagLibrary aprilTagLibrary = new AprilTagLibrary.Builder()
                .addTag(1, "BlueLeft", 2.0, DistanceUnit.INCH)
                .addTag(2, "BlueCenter", 2.0, DistanceUnit.INCH)
                .addTag(3, "BlueRight", 2.0, DistanceUnit.INCH)
                .addTag(4, "RedLeft", 2.0, DistanceUnit.INCH)
                .addTag(5, "RedCenter", 2.0, DistanceUnit.INCH)
                .addTag(6, "RedRight", 2.0, DistanceUnit.INCH)
                .build();

        aprilTagProcessor = new AprilTagProcessorImpl(902.125, 902.125, 604.652, 368.362, DistanceUnit.INCH, AngleUnit.DEGREES, aprilTagLibrary, true, true, true, true, AprilTagProcessor.TagFamily.TAG_36h11, 1); // Used for managing the AprilTag detection process.
        colorDetectionProcessor = new ColorDetectionProcessor(AllianceHelper.Alliance.RED); // Used for managing the color detection process.
        whiteDetectionProcessor = new WhiteDetectionProcessor(); // Used for managing the white detection process.

        List<Integer> myPortalList = JavaUtil.makeIntegerList(VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL));
        int portal_1_View_ID = (Integer) JavaUtil.inListGet(myPortalList, JavaUtil.AtMode.FROM_START, 0, false);
        int portal_2_View_ID = (Integer) JavaUtil.inListGet(myPortalList, JavaUtil.AtMode.FROM_START, 1, false);

        aprilTagPortal = new VisionPortal.Builder()
                .setCamera(backCamera)
                .setCameraResolution(new android.util.Size(1280, 720))
                .addProcessor(aprilTagProcessor)
                .setLiveViewContainerId(portal_2_View_ID)
                .build();
        aprilTagAlignment = new AprilTagAlignmentAuto(backCamera, 0.0, 12.0, 0.0, AllianceHelper.alliance, aprilTagProcessor);

        colorPortal = new VisionPortal.Builder()
                .setCamera(frontCamera)
                .setCameraResolution(new android.util.Size(320, 240))
                .addProcessors(colorDetectionProcessor, whiteDetectionProcessor)
                .setLiveViewContainerId(portal_1_View_ID)
                .build();

        aprilTagPortal.stopStreaming();
        colorPortal.setProcessorEnabled(colorDetectionProcessor, true);
        colorPortal.setProcessorEnabled(whiteDetectionProcessor, false);
    }
    private void detectPurplePath() {
        purplePixelPath = colorDetectionProcessor.getPosition();
        telemetry.addData("Purple Pixel Path: ", purplePixelPath.toString());
    }

    public void buildAuto() {
        switch (startPosition) {
            case BLUE_CLOSE:
                initPose = new Pose2d(16, 63, Math.toRadians(270));
                purplePose = new Pose2d(-63, -48, Math.toRadians(0));
                purpleTangent = Math.toRadians(210);
                initTangent = Math.toRadians(300);
                beforeStackPose = new Pose2d(24,11, Math.toRadians(180));
                whitePixelStackPose = new Pose2d(-57,-6.5, Math.toRadians(180));//This had better be the same every time TY
                postLowerWhitePose = new Pose2d(28, 10, Math.toRadians(180));
                preLowerWhiteTangent = 225;
                switch (purplePixelPath) {
                    case LEFT:
                        purplePose = new Pose2d(11,-32, Math.toRadians(180));
                        aprilTagPose = new Pose2d(-63, -42, Math.toRadians(0)); // TODO see below
                        backboardPose = new Pose2d(-65, -42, Math.toRadians(0)); // TODO see below
                        backboardApriltagX = -6;
                        centerBackup = 1;
                        break;
                    case CENTER:
                        purplePose = new Pose2d(23,-24.2, Math.toRadians(180));
                        aprilTagPose = new Pose2d(-63, -48, Math.toRadians(0)); // TODO see below
                        backboardPose = new Pose2d(-65, -48, Math.toRadians(0)); // TODO see below
                        backboardApriltagX = 0;
                        centerBackup = 1;
                        break;
                    case RIGHT:
                        purplePose = new Pose2d(25,-32, Math.toRadians(180));
                        aprilTagPose = new Pose2d(-63, -54, Math.toRadians(0)); // TODO see below
                        backboardPose = new Pose2d(-65, -54, Math.toRadians(0)); // TODO see below
                        backboardApriltagX = 6;
                        centerBackup = 1;
                        break;
                }
                break;
            case RED_CLOSE:
                initPose = new Pose2d(11, -63, Math.toRadians(90));
                purplePose = new Pose2d(11, -25, Math.toRadians(180));
                purpleTangent = Math.toRadians(80);
                initTangent = Math.toRadians(60);
                beforeStackPose = new Pose2d(24,-10, Math.toRadians(180));
                whitePixelStackPose = new Pose2d(-57,-10, Math.toRadians(180));
                preWhitePose = new Pose2d(24, -10, Math.toRadians(180));
                postLowerWhitePose = new Pose2d(28, -10, Math.toRadians(180));
                preLowerWhiteTangent = 135;
                switch (purplePixelPath) {
                    case LEFT:
                        purplePose = new Pose2d(11,-32, Math.toRadians(180));
                        aprilTagPose = new Pose2d(52, -39, Math.toRadians(180)); // *this is below* TODO adjust for april tag estimate to get tag in frame
                        backboardPose = new Pose2d(52, -39, Math.toRadians(180)); // TODO see below
                        backboardApriltagX = -6;
                        centerBackup = 3.5; // FIX THIS POOP
                        break;
                    case CENTER:
                        purplePose = new Pose2d(23,-24.2, Math.toRadians(180));
                        aprilTagPose = new Pose2d(52, -39, Math.toRadians(180)); // TODO adjust for april tag estimate to get tag in frame
                        backboardPose = new Pose2d(52, -39, Math.toRadians(180)); // TODO see below
                        backboardApriltagX = 0;
                        centerBackup = 3.5; // FIX THIS POOP
                        break;
                    case RIGHT:
                        purplePose = new Pose2d(12.5,-30, Math.toRadians(180));
                        aprilTagPose = new Pose2d(52, -39, Math.toRadians(180)); // TODO adjust for april tag estimate to get tag in frame
                        backboardPose = new Pose2d(52, -39, Math.toRadians(180)); // TODO see below
                        backboardApriltagX = 6;
                        centerBackup = 3.5; // FIX THIS POOP
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
        intake.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMotorTargetPosition(250);
        intake.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setMotorPower(0.5);
        intake.update();
    }

    private void outtake() {
        outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.OUTSIDE);
        setSlideHeight(-900);
    }

    private void outtakeIn() {
        if(outtake.getOuttakePosition() == Outtake.OuttakePositions.OUTSIDE)
            setSlideHeight(-900);
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
        //******select start pose*****
        while (!positionFound && !isStopRequested()) {
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
                positionFound = true;
            }
            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                startPosition = START_POSITION.BLUE_FAR;
                AllianceHelper.alliance = AllianceHelper.Alliance.BLUE;
                positionFound = true;
            }
            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                startPosition = START_POSITION.RED_FAR;
                AllianceHelper.alliance = AllianceHelper.Alliance.RED;
                positionFound = true;
            }
            if (gamepad1.dpad_right || gamepad2.dpad_left) {
                startPosition = START_POSITION.RED_CLOSE;
                AllianceHelper.alliance = AllianceHelper.Alliance.RED;
                positionFound = true;
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
