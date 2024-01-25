package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
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

@Autonomous(name = "AutoRegionals", preselectTeleOp = "Main Teleop", group = "a")
public class AutoRegionals extends LinearOpMode {
    //ROBOT STARTING POSITIONS
    private enum StartPosition {
        RED_CLOSE,
        RED_FAR,
        BLUE_CLOSE,
        BLUE_FAR
    }
    private enum Path {
        PLACEMENT,
        INSIDE,
        OUTSIDE
    }

    private StartPosition startPosition = StartPosition.RED_CLOSE;
    private ColorDetectionProcessor.StartingPosition purplePixelPath = ColorDetectionProcessor.StartingPosition.CENTER;
    private Path path;

//    private Datalog datalog; //TELEMETRY

    //DETECTION
    private ColorDetectionProcessor colorDetectionProcessor;
    private AprilTagAlignmentProcessor aprilTagProcessorBack;
    private AprilTagAlignmentProcessor aprilTagProcessorFront;
    private VisionPortal frontCameraPortal;
    private VisionPortal backCameraPortal;

    //HARDWARE
    private CogchampDrive drive;
    private Intake intake;
    private Outtake outtake;
    private Slide slide;
    private VoltageSensor battery;

    // TRAJECTORIES
    private TrajectorySequence init; // INIT THEN GO TO BACKBOARD
    private TrajectorySequence backboardToSpike; // BACKBOARD TO SPIKE
    private TrajectorySequence backboardToWhite; //BACKBOARD TO STACK
    private TrajectorySequence spikeToWhite; //SPIKE TO STACK
    private TrajectorySequence spikeToPark; //SPIKE TO PARK
    private TrajectorySequence whiteToBackboard;// STACK BACK TO BACKBOARD
    private TrajectorySequence backboardToPark; //PARK AFTER SCORING STACK
    private TrajectorySequence spikeToBackboard; //SPIKE TO BACKBOARD

    //POSITIONS & TANGENTS
    private Pose2d initPose; //INITIAL POSITION
    private Pose2d spikePose; //SPIKE POSITION
    private Pose2d wingTruss; //WING TRUSS POSITION
    private Pose2d boardTruss; //BOARD TRUSS POSITION
    private Pose2d parkPose; //Parking Position
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
        selectStartingPosition();
        initialize();

        while(!isStarted() && !isStopRequested())
            initLoop();

        waitForStart();

        startAuto();

        while(opModeIsActive() && !isStopRequested())
            autoLoop();
    }

    private void initialize() {
        //BULK READS
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        //DEFINE HARDWARE
        drive = new CogchampDrive(hardwareMap);
        slide = new Slide(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap, slide);
        battery = hardwareMap.voltageSensor.get("Control Hub");

//        datalog = new Datalog("AutoDatalog"+ SimpleDateFormat.getDateTimeInstance().format(new Date()));

//        datalog.opModeStatus.set("INIT");
//        datalog.battery.set(battery.getVoltage());
//        datalog.writeLine();

        //SET INITIAL HARDWARE STATES
        outtake.setGateClosed(true);
        outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.INSIDE);
        outtake.update();
        intake.setServoPosition(Intake.IntakePositions.INIT);
        intake.update();

        //INITIALIZE DETECTION PORTALS
        initPortals();

        backCameraPortal.stopLiveView();
    }

    private void initLoop() {
        detectPurplePath();
        telemetry.addData("Selected Auto: ", startPosition.toString());
        telemetry.addData("Detected Path: ", purplePixelPath.toString());
        telemetry.update();
    }

    private void startAuto() {
        frontCameraPortal.stopStreaming();
        backCameraPortal.resumeLiveView();
        telemetry.clear();
        buildAuto();
        drive.setPoseEstimate(initPose);

        //CYCLE PATHS
        if(path == Path.OUTSIDE) {
            whiteToBackboard = drive.trajectorySequenceBuilder(spikePose)
                    .addTemporalMarker(this::intake)
                    .addTemporalMarker(this::outtakeTransfer)
                    .forward(2)
                    .waitSeconds(1)
                    .setTangent(0)
                    .addTemporalMarker(this::stopIntake)
                    .addTemporalMarker(this::transfer)
                    .addTemporalMarker(() -> {
                        intake.intakePower(-1.0);
                    })
                    .splineToSplineHeading(wingTruss, Math.toRadians(0))
                    .addTemporalMarker(() -> {
                        intake.intakePower(0.0);
                    })
                    .splineToSplineHeading(boardTruss, Math.toRadians(0))
                    .addTemporalMarker(this::outtake)
                    .splineToLinearHeading(backboardPose, Math.toRadians(30))
                    .addTemporalMarker(() -> {
                        aprilTagProcessorBack.setTargetPose(backboardPose);
                        alignToApriltagBackboard();
                        drive.setPoseEstimate(backboardPose); //TODO add apriltag errors
                        if (path != Path.PLACEMENT)
                            drive.followTrajectorySequenceAsync(backboardToWhite);
                        else
                            drive.followTrajectorySequenceAsync(backboardToPark);
                    })
                    .build();
            backboardToWhite = drive.trajectorySequenceBuilder(backboardPose)
                    .back(2)
                    .addTemporalMarker(this::drop)
                    .addTemporalMarker(this::outtakeIn)
                    .waitSeconds(0.5)
                    .setTangent(Math.toRadians(210.0))
                    .splineToLinearHeading(PoseHelper.boardTrussOutsideRed, Math.toRadians(180.0))
                    .splineToLinearHeading(PoseHelper.wingTrussOutsideRed, Math.toRadians(180.0))
                    .splineToLinearHeading(PoseHelper.apriltagStackRed, Math.toRadians(180.0))
                    .addTemporalMarker(() -> {
                        alignToApriltagStack();
                        drive.followTrajectorySequenceAsync(whiteToBackboard);
                    })
                    .waitSeconds(.1)
                    .build();
            backboardToPark = drive.trajectorySequenceBuilder(backboardPose)
                    .setTangent(Math.toRadians(120.0))
                    .splineToLinearHeading(parkPose, Math.toRadians(180.0))
                    .build();
        }
        else {
            //TODO INSIDE PATH CODE
        }

        //INITIAL PATHS
        if(startPosition == StartPosition.RED_CLOSE || startPosition == StartPosition.BLUE_CLOSE) {
            init = drive.trajectorySequenceBuilder(initPose)
                    .splineToLinearHeading(backboardPose, Math.toRadians(180.0))
                    .addTemporalMarker(() -> {
                        alignToApriltagBackboard(); //TODO add apriltag errors
                        drive.setPoseEstimate(backboardPose);
                        drive.followTrajectorySequenceAsync(backboardToSpike); })
                    .build();
            backboardToSpike = drive.trajectorySequenceBuilder(backboardPose)
                    .setTangent(Math.toRadians(180.0))
                    .splineToLinearHeading(spikePose, Math.toRadians(180.0))
                    .addTemporalMarker(this::outtakePurple)
                    .addTemporalMarker(() -> {
                        if(path != Path.PLACEMENT)
                            drive.followTrajectorySequenceAsync(spikeToWhite);
                        else
                            drive.followTrajectorySequenceAsync(spikeToPark);
                    })
                    .build();
            spikeToPark = drive.trajectorySequenceBuilder(spikePose)
                    //TODO PARK
                    .build();
            spikeToWhite = drive.trajectorySequenceBuilder(spikePose)
                    .setTangent(Math.toRadians(270))
                    .splineToLinearHeading(boardTruss, Math.toRadians(180.0))
                    .splineToLinearHeading(wingTruss, Math.toRadians(180.0))
                    .splineToLinearHeading(aprilTagPose, Math.toRadians(180.0))
                    .addTemporalMarker(() -> {
                        alignToApriltagStack();
                        drive.followTrajectorySequenceAsync(whiteToBackboard);
                    })
                    .build();
        }
        else { //FAR AUTO
            init = drive.trajectorySequenceBuilder(initPose)
                    .addTemporalMarker(() -> { intake.setServoPosition(Intake.IntakePositions.INTAKE); })
                    .splineToLinearHeading(spikePose, spikePose.getHeading())
                    .addTemporalMarker(() -> {
                        drive.followTrajectorySequenceAsync(spikeToWhite);
                    })
                    .build();
            spikeToWhite = drive.trajectorySequenceBuilder(spikePose)
                    .addTemporalMarker(this::outtakePurple)
                    .back(4)
                    .addTemporalMarker(() -> {
                        intake.setServoPosition(Intake.IntakePositions.FIVE);
                    })
                    .setTangent(Math.toRadians(180))
                    .splineToLinearHeading(PoseHelper.apriltagStackRed, Math.toRadians(180.0))
                    .addTemporalMarker(() -> {
                        alignToApriltagStack();
                        drive.followTrajectorySequenceAsync(whiteToBackboard);
                    })
                    .build();
        }

        drive.followTrajectorySequenceAsync(init);
    }

    private void autoLoop() {
        drive.update();
        intake.update();
        outtake.update();
        slide.update();
    }

    private void buildAuto() {
        switch(AllianceHelper.alliance) {
            case RED:
                backboardPose = PoseHelper.backboardRed;
                switch(path) {
                    case OUTSIDE:
                        parkPose = PoseHelper.parkPoseRedOutside;
                        wingTruss = PoseHelper.wingTrussOutsideRed;
                        boardTruss = PoseHelper.boardTrussOutsideRed;
                        break;
                    case INSIDE:
                        parkPose = PoseHelper.parkPoseRedInside;
                        wingTruss = PoseHelper.wingTrussInsideRed;
                        boardTruss = PoseHelper.boardTrussInsideRed;
                        break;
                }
                break;
            case BLUE:
                backboardPose = PoseHelper.backboardBlue;
                switch(path) {
                    case OUTSIDE:
                        parkPose = PoseHelper.parkPoseBlueOutside;
                        wingTruss = PoseHelper.wingTrussOutsideBlue;
                        boardTruss = PoseHelper.boardTrussOutsideBlue;
                        break;
                    case INSIDE:
                        parkPose = PoseHelper.parkPoseBlueInside;
                        wingTruss = PoseHelper.wingTrussInsideBlue;
                        boardTruss = PoseHelper.boardTrussInsideBlue;
                        break;
                }
                break;
        }
        switch(startPosition) {
            case RED_CLOSE:
                initPose = PoseHelper.initCloseRed;
                switch(purplePixelPath) {
                    case LEFT:
                        spikePose = PoseHelper.closeSpikeLeftRed;
                        break;
                    case CENTER:
                        spikePose = PoseHelper.closeSpikeCenterRed;
                        break;
                    case RIGHT:
                        spikePose = PoseHelper.closeSpikeRightRed;
                        break;
                }
                break;
            case RED_FAR:
                initPose = PoseHelper.initFarRed;
                switch(purplePixelPath) {
                    case LEFT:
                        spikePose = PoseHelper.farSpikeLeftRed;
                        break;
                    case CENTER:
                        spikePose = PoseHelper.farSpikeCenterRed;
                        break;
                    case RIGHT:
                        spikePose = PoseHelper.farSpikeRightRed;
                        break;
                }
                break;
            case BLUE_CLOSE:
                initPose = PoseHelper.initCloseBlue;
                switch(purplePixelPath) {
                    case LEFT:
                        spikePose = PoseHelper.closeSpikeLeftBlue;
                        break;
                    case CENTER:
                        spikePose = PoseHelper.closeSpikeCenterBlue;
                        break;
                    case RIGHT:
                        spikePose = PoseHelper.closeSpikeRightBlue;
                        break;
                }
                break;
            case BLUE_FAR:
                initPose = PoseHelper.initFarBlue;
                switch(purplePixelPath) {
                    case LEFT:
                        spikePose = PoseHelper.farSpikeLeftBlue;
                        break;
                    case CENTER:
                        spikePose = PoseHelper.farSpikeCenterBlue;
                        break;
                    case RIGHT:
                        spikePose = PoseHelper.farSpikeRightBlue;
                        break;
                }
                break;
        }
    }

    private void initPortals() {
        CameraName backCamera = hardwareMap.get(WebcamName.class, "Webcam 1");
        CameraName frontCamera = hardwareMap.get(WebcamName.class, "Webcam 2");

        aprilTagProcessorBack = new AprilTagAlignmentProcessor(AprilTagAlignmentProcessor.CameraType.BACK, AllianceHelper.alliance == AllianceHelper.Alliance.RED ? PoseHelper.backboardRed : PoseHelper.backboardBlue); // Used for managing the april tag detection process.
        aprilTagProcessorFront = new AprilTagAlignmentProcessor(AprilTagAlignmentProcessor.CameraType.FRONT, AllianceHelper.alliance == AllianceHelper.Alliance.RED ? PoseHelper.apriltagStackRed : PoseHelper.apriltagStackBlue); // Used for managing the april tag detection process.
        colorDetectionProcessor = new ColorDetectionProcessor(AllianceHelper.alliance); // Used for managing the color detection process.

        List<Integer> portalList = JavaUtil.makeIntegerList(VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL));
        int frontPortalId = (Integer) JavaUtil.inListGet(portalList, JavaUtil.AtMode.FROM_START, 0, false);
        int backPortalId = (Integer) JavaUtil.inListGet(portalList, JavaUtil.AtMode.FROM_START, 1, false);

        frontCameraPortal = new VisionPortal.Builder()
                .setCamera(frontCamera)
                .setCameraResolution(new android.util.Size(1280, 720))
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

    private void detectPurplePath() { purplePixelPath = colorDetectionProcessor.getPosition(); }

    private void alignToApriltagBackboard() {
        double currentTime = getRuntime();
        while(!isStopRequested()) {
            aprilTagProcessorBack.update();
            aprilTagProcessorBack.alignRobot(drive);

            telemetry.addData("x error","%5.1f inches", aprilTagProcessorBack.getYError());
            telemetry.addData("y error","%5.1f inches", aprilTagProcessorBack.getXError());
            telemetry.addData("heading error","%3.0f degrees", aprilTagProcessorBack.getHeadingError());
            telemetry.addData("drivetrain power", drive.getPoseEstimate());
            telemetry.update();
            intake.update();
            slide.update();
            outtake.update();

            if(getRuntime()-currentTime > 2 || aprilTagProcessorBack.robotAligned()) break;
        }
    }

    private void alignToApriltagStack() {
        double currentTime = getRuntime();
        while(!isStopRequested()) {
            aprilTagProcessorFront.update();
            aprilTagProcessorFront.alignRobot(drive);

            telemetry.addData("x error","%5.1f inches", aprilTagProcessorBack.getYError());
            telemetry.addData("y error","%5.1f inches", aprilTagProcessorBack.getXError());
            telemetry.addData("heading error","%3.0f degrees", aprilTagProcessorBack.getHeadingError());
            telemetry.addData("drivetrain power", drive.getPoseEstimate());
            telemetry.update();
            intake.update();
            slide.update();
            outtake.update();

            if(getRuntime()-currentTime > 2 || aprilTagProcessorBack.robotAligned()) break;
        }
    }

    //INTAKE METHODS
    private void intake() { intake.intakePower(1.0); intake.update(); }
    private void stopIntake() {
        intake.intakePower(0.0);
        intake.setBoosterServoPower(-1.0);
        intake.update();
    }
    private void transfer() { intake.transfer(); intake.update(); }
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

    //OUTTAKE METHODS
    private void outtake() { outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.OUTSIDE); if(slide.getAvgPosition() >= -900) setSlideHeight(-900); }
    private void outtakeIn() { if(outtake.getOuttakePosition() == Outtake.OuttakePositions.OUTSIDE) setSlideHeight(-1200); outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.INSIDE); }
    private void outtakeTransfer() { outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.TRANSFER); outtake.update(); }
    private void drop() { outtake.setGateClosed(false); }

    private void setSlideHeight(int height) { slide.setTargetPosition(height); slide.setMode(DcMotor.RunMode.RUN_TO_POSITION); slide.setPower(1.0); }

    public void selectStartingPosition() {
        while (!isStopRequested()) {
            telemetry.addLine("             [15118 AUTO INITIALIZED]");
            telemetry.addLine("-------------------------------------------------");
            telemetry.addLine("Select Autonomous Starting Position using DPAD Keys");
            telemetry.addData("    Blue Close   ", "(^)");
            telemetry.addData("    Blue Far     ", "(v)");
            telemetry.addData("    Red Far      ", "(<)");
            telemetry.addData("    Red Close    ", "(>)");

            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                startPosition = StartPosition.BLUE_CLOSE;
                AllianceHelper.alliance = AllianceHelper.Alliance.BLUE;
                break;
            }
            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                startPosition = StartPosition.BLUE_FAR;
                AllianceHelper.alliance = AllianceHelper.Alliance.BLUE;
                break;
            }
            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                startPosition = StartPosition.RED_FAR;
                AllianceHelper.alliance = AllianceHelper.Alliance.RED;
                break;
            }
            if (gamepad1.dpad_right || gamepad2.dpad_right) {
                startPosition = StartPosition.RED_CLOSE;
                AllianceHelper.alliance = AllianceHelper.Alliance.RED;
                break;
            }
            telemetry.update();
        }
        while (!isStopRequested()) {
            telemetry.addLine("             [15118 AUTO INITIALIZED]");
            telemetry.addLine("-------------------------------------------------");
            telemetry.addLine(" Selected " + startPosition.toString() + " Starting Position.");
            telemetry.addLine();
            telemetry.addLine("Select Autonomous Path using DPAD Keys");
            telemetry.addData("     Inside      ", "(^)");
            telemetry.addData("     Outside     ", "(v)");

            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                path = Path.INSIDE;
                break;
            }
            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                path = Path.OUTSIDE;
                break;
            }
            telemetry.update();
        }
        telemetry.clear();
    }
    public static class Datalog {
        private final Datalogger datalogger;
        public Datalogger.GenericField opModeStatus = new Datalogger.GenericField("OpModeStatus");
        public Datalogger.GenericField loopCounter  = new Datalogger.GenericField("Loop Counter");
        public Datalogger.GenericField xError       = new Datalogger.GenericField("X Error");
        public Datalogger.GenericField yError       = new Datalogger.GenericField("Y Error");
        public Datalogger.GenericField headingError = new Datalogger.GenericField("Heading Error");
        public Datalogger.GenericField battery      = new Datalogger.GenericField("Battery");
        public Datalog(String name) {
            datalogger = new Datalogger.Builder()
                    .setFilename(name)
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)
                    .setFields( opModeStatus, loopCounter, xError, yError, headingError, battery )
                    .build();
        }
        public void writeLine()
        {
            datalogger.writeLine();
        }
    }
}