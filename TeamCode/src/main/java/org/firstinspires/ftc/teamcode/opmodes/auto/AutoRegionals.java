package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.icu.text.SimpleDateFormat;

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

import java.util.Date;
import java.util.List;

@Autonomous(name = "AutoRegionals", preselectTeleOp = "Main Teleop", group = "a")
public class AutoRegionals extends LinearOpMode {
    private Datalog datalog; //TELEMETRY

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
    private TrajectorySequence whiteToBackboardYellow; //STACK TO BACKBOARD WITH YELLOW
    private TrajectorySequence dropYellow; //DROP YELLOW
    private TrajectorySequence dropWhite; //DROP WHITE
    private TrajectorySequence backboardToPark; //PARK AFTER SCORING STACK

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

        datalog = new Datalog("AutoDatalog"+ SimpleDateFormat.HOUR24_MINUTE_SECOND);

        datalog.opModeStatus.set("INIT");
        datalog.battery.set(battery.getVoltage());
        datalog.writeLine();

        //SET INITIAL HARDWARE STATES
        outtake.setGateClosed(true);
        outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.INSIDE);
        outtake.update();
        intake.setServoPosition(Intake.IntakePositions.INIT);
        intake.update();

        //INITIALIZE DETECTION PORTALS
        initPortals();
    }

    private void initLoop() {
        detectPurplePath();
        telemetry.addData("Selected Auto: ", StartPosition.startPosition.toString());
        telemetry.addData("Detected Path: ", ColorDetectionProcessor.position.toString());
        telemetry.update();

        if(backCameraPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            backCameraPortal.stopStreaming();
        }
    }

    private void startAuto() {
        frontCameraPortal.stopLiveView();
        backCameraPortal.resumeStreaming();
        telemetry.clear();
        PoseHelper.buildAuto();
        drive.setPoseEstimate(PoseHelper.initPose);

        //INITIAL PATHS
        //CLOSE AUTO
        if(StartPosition.startPosition == StartPosition.StartPos.RED_CLOSE || StartPosition.startPosition == StartPosition.StartPos.BLUE_CLOSE) {
            init = drive.trajectorySequenceBuilder(PoseHelper.initPose)
                    .addTemporalMarker(this::outtake)
                    .splineToLinearHeading(PoseHelper.backboardPose, Math.toRadians(0))
                    .addTemporalMarker(() -> {
                        alignToApriltagBackboard();
                        drive.setPoseEstimate(aprilTagProcessorBack.getPoseEstimate());
                        drive.followTrajectorySequenceAsync(backboardToSpike); })
                    .build();
            backboardToSpike = drive.trajectorySequenceBuilder(PoseHelper.backboardPose)
                    .back(6)
                    .addTemporalMarker(this::drop)
                    .waitSeconds(0.2)
                    .addTemporalMarker(() -> setSlideHeight(-1100))
                    .waitSeconds(0.2)
                    .forward(6)
                    .addTemporalMarker(this::outtakeIn)
                    .addTemporalMarker(() -> intake.setServoPosition(Intake.IntakePositions.INTAKE))
                    .setTangent(Math.toRadians(180.0))
                    .splineToLinearHeading(PoseHelper.spikePose, Math.toRadians(180.0))
                    .back(4)
                    .addTemporalMarker(this::outtakePurple)
                    .addTemporalMarker(() -> {
                        if(Paths.path != Paths.Path.PLACEMENT)
                            drive.followTrajectorySequenceAsync(spikeToWhite);
                        else
                            drive.followTrajectorySequenceAsync(spikeToPark);
                    })
                    .build();
            spikeToPark = drive.trajectorySequenceBuilder(PoseHelper.spikePose)
                    .splineToLinearHeading(PoseHelper.parkPose, Math.toRadians(180.0))
                    //TODO PARK
                    .build();
            spikeToWhite = drive.trajectorySequenceBuilder(PoseHelper.spikePose)
                    .setTangent(Math.toRadians(Paths.path == Paths.Path.OUTSIDE ? 270.0 : 90.0 * PoseHelper.allianceAngleMultiplier))
                    .splineToLinearHeading(PoseHelper.boardTruss, Math.toRadians(180.0))
                    .addTemporalMarker(() -> intake.setServoPosition(Intake.IntakePositions.FOUR))
                    .setTangent(Math.toRadians(180))
                    .splineToLinearHeading(PoseHelper.wingTruss, Math.toRadians(180.0))
                    .splineToLinearHeading(PoseHelper.stackPose, Math.toRadians(180.0))
                    .addTemporalMarker(() -> {
                        alignToApriltagStack();
                        drive.followTrajectorySequenceAsync(whiteToBackboard);
                    })
                    .build();
        }
        else { //FAR AUTO
            init = drive.trajectorySequenceBuilder(PoseHelper.initPose)
                    .addTemporalMarker(() -> intake.setServoPosition(Intake.IntakePositions.INTAKE))
                    .splineToLinearHeading(PoseHelper.spikePose, PoseHelper.spikePose.getHeading())
                    .addTemporalMarker(() -> drive.followTrajectorySequenceAsync(spikeToWhite))
                    .build();
            spikeToWhite = drive.trajectorySequenceBuilder(PoseHelper.spikePose)
                    .addTemporalMarker(this::outtakePurple)
                    .back(PoseHelper.purpleBackDistanceFar)
                    .addTemporalMarker(() -> intake.setServoPosition(Intake.IntakePositions.FIVE))
                    .setTangent(Math.toRadians(PoseHelper.toWhiteStackTangentFar))
                    .splineToLinearHeading(PoseHelper.stackPose, Math.toRadians(PoseHelper.toWhiteStackTangentFar))
                    .addTemporalMarker(() -> {
                        alignToApriltagStack();
                        drive.followTrajectorySequenceAsync(whiteToBackboardYellow);
                    })
                    .build();
            whiteToBackboardYellow = drive.trajectorySequenceBuilder(PoseHelper.stackPose)
                    .addTemporalMarker(this::intake)
                    .forward(2)
                    .back(2)
                    .addTemporalMarker(this::stopIntake)
                    .splineToLinearHeading(PoseHelper.wingTruss, Math.toRadians(0))
                    .addTemporalMarker(() -> { intake.setBoosterServoPower(0.0); })
                    .splineToLinearHeading(PoseHelper.boardTruss, Math.toRadians(0))
                    .addTemporalMarker(() -> setSlideHeight(-1050))
                    .addTemporalMarker(this::outtake)
                    .splineToLinearHeading(PoseHelper.backboardPose, Math.toRadians(30.0 * PoseHelper.allianceAngleMultiplier))
                    .addTemporalMarker(() -> {
                        datalog.opModeStatus.set("APRILTAG BACKBOARD 1");
                        datalog.writeLine();
                        alignToApriltagBackboard();
                        drive.setPoseEstimate(aprilTagProcessorBack.getPoseEstimate());
                        buildDropYellow();
                        drive.followTrajectorySequenceAsync(dropYellow);
                    })
                    .build();
        }

        //CYCLE PATHS
        whiteToBackboard = drive.trajectorySequenceBuilder(PoseHelper.stackPose)
                .addTemporalMarker(this::intake)
                .addTemporalMarker(this::outtakeTransfer)
                .forward(2)
                .waitSeconds(0.5)
                .addTemporalMarker(this::stopIntake)
                .splineToLinearHeading(PoseHelper.wingTruss, Math.toRadians(0))
                .addTemporalMarker(this::transfer)
                .splineToLinearHeading(PoseHelper.boardTruss, Math.toRadians(0))
                .addTemporalMarker(this::outtake)
                .splineToLinearHeading(PoseHelper.backboardPose, Math.toRadians(Paths.path == Paths.Path.OUTSIDE ? 30.0 : -30.0 * PoseHelper.allianceAngleMultiplier))
                .addTemporalMarker(() -> {
                    alignToApriltagBackboard();
                    drive.setPoseEstimate(aprilTagProcessorBack.getPoseEstimate());
                    if (Paths.path != Paths.Path.PLACEMENT)
                        drive.followTrajectorySequenceAsync(backboardToWhite);
                    else
                        drive.followTrajectorySequenceAsync(backboardToPark);
                })
                .build();
        backboardToPark = drive.trajectorySequenceBuilder(PoseHelper.backboardPose)
                .setTangent(Math.toRadians(Paths.path == Paths.Path.OUTSIDE ? 150.0 : -150.0 * PoseHelper.allianceAngleMultiplier))
                .splineToLinearHeading(PoseHelper.parkPose, Math.toRadians(180.0))
                .build();

        drive.followTrajectorySequenceAsync(init);
    }

    private void buildDropYellow() {
        dropYellow = drive.trajectorySequenceBuilder(aprilTagProcessorBack.getPoseEstimate()) //TODO CHANGE TO APRILTAG POSE ESTIMATE IF POSSIBLE
                .back(6)
                .addTemporalMarker(this::drop)
                .waitSeconds(0.2)
                .addTemporalMarker(()->setSlideHeight(-1400))
                .waitSeconds(0.2)
                .forward(6)
                .addTemporalMarker(this::outtakeTransfer)
                .waitSeconds(0.8)
                .addTemporalMarker(this::transfer)
                .waitSeconds(1.6)
                .addTemporalMarker(this::outtake)
                .addTemporalMarker(() -> setSlideHeight(-1400))
                .addTemporalMarker(() -> {
                    //DEFAULTS TO RIGHT SIDE IF POSSIBLE
                    if(PoseHelper.backboardPose == PoseHelper.backboardRightRed || PoseHelper.backboardPose == PoseHelper.backboardRightBlue) {
                        if (AllianceHelper.alliance == AllianceHelper.Alliance.RED)
                            PoseHelper.backboardPose = PoseHelper.backboardCenterRed;
                        else
                            PoseHelper.backboardPose = PoseHelper.backboardCenterBlue;
                    }
                    else {
                        if (AllianceHelper.alliance == AllianceHelper.Alliance.RED)
                            PoseHelper.backboardPose = PoseHelper.backboardRightRed;
                        else
                            PoseHelper.backboardPose = PoseHelper.backboardRightBlue;
                    }
                    datalog.opModeStatus.set("APRILTAG BACKBOARD 2");
                    datalog.writeLine();
                    alignToApriltagBackboard();
                    drive.setPoseEstimate(aprilTagProcessorBack.getPoseEstimate());
                    buildBackboardToWhite();
                    drive.followTrajectorySequenceAsync(backboardToWhite);
                })
                .build();

    }

    private void buildBackboardToWhite() {
        backboardToWhite = drive.trajectorySequenceBuilder(aprilTagProcessorBack.getPoseEstimate())
                .back(6)
                .addTemporalMarker(this::drop)
                .waitSeconds(0.2)
                .addTemporalMarker(() -> setSlideHeight(-1400))
                .waitSeconds(0.2)
                .addTemporalMarker(this::outtakeIn)
                .waitSeconds(0.2)
                .setTangent(Math.toRadians(210.0 * PoseHelper.allianceAngleMultiplier))
                .splineToLinearHeading(PoseHelper.boardTruss, Math.toRadians(180.0))
                .splineToLinearHeading(PoseHelper.wingTruss, Math.toRadians(180.0))
                .splineToLinearHeading(PoseHelper.stackPose, Math.toRadians(PoseHelper.toWhiteStackTangentFar))
                .addTemporalMarker(() -> {
                    alignToApriltagStack();
                    drive.followTrajectorySequenceAsync(whiteToBackboard);
                })
                .build();
    }

    private void autoLoop() {
        drive.update();
        intake.update();
        outtake.update();
        slide.update();
    }

    private void initPortals() {
        CameraName backCamera = hardwareMap.get(WebcamName.class, "Webcam 1");
        CameraName frontCamera = hardwareMap.get(WebcamName.class, "Webcam 2");

        aprilTagProcessorBack = new AprilTagAlignmentProcessor(AprilTagAlignmentProcessor.CameraType.BACK, AllianceHelper.alliance == AllianceHelper.Alliance.RED ? PoseHelper.backboardCenterRed : PoseHelper.backboardCenterBlue); // Used for managing the april tag detection process.
        aprilTagProcessorFront = new AprilTagAlignmentProcessor(AprilTagAlignmentProcessor.CameraType.FRONT, AllianceHelper.alliance == AllianceHelper.Alliance.RED ? PoseHelper.apriltagStackRed : PoseHelper.apriltagStackBlue); // Used for managing the april tag detection process.
        colorDetectionProcessor = new ColorDetectionProcessor(AllianceHelper.alliance); // Used for managing the color detection process.

        List<Integer> portalList = JavaUtil.makeIntegerList(VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL));
        int frontPortalId = (Integer) JavaUtil.inListGet(portalList, JavaUtil.AtMode.FROM_START, 0, false);
        int backPortalId = (Integer) JavaUtil.inListGet(portalList, JavaUtil.AtMode.FROM_START, 1, false);

        frontCameraPortal = new VisionPortal.Builder()
                .setCamera(frontCamera)
                .setCameraResolution(new android.util.Size(640, 480))
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

    private void detectPurplePath() { ColorDetectionProcessor.position = colorDetectionProcessor.getPosition(); }

    private void alignToApriltagBackboard() {
        aprilTagProcessorBack.setTargetPose(PoseHelper.backboardPose);
        double currentTime = getRuntime();
        while(!isStopRequested()) {
            aprilTagProcessorBack.update();

            datalog.xError.set(aprilTagProcessorBack.getPoseError().getX());
            datalog.yError.set(aprilTagProcessorBack.getPoseError().getY());
            datalog.headingError.set(aprilTagProcessorBack.getPoseError().getHeading());
            datalog.xEstimate.set(aprilTagProcessorBack.getPoseEstimate().getX());
            datalog.yEstimate.set(aprilTagProcessorBack.getPoseEstimate().getY());
            datalog.headingEstimate.set(aprilTagProcessorBack.getPoseEstimate().getHeading());
            datalog.writeLine();

            if(getRuntime()-currentTime > 1) break;

            aprilTagProcessorBack.alignRobot(drive);

            telemetry.addData("x error","%5.1f inches", aprilTagProcessorBack.getPoseError().getX());
            telemetry.addData("y error","%5.1f inches", aprilTagProcessorBack.getPoseError().getY());
            telemetry.addData("heading error","%3.0f degrees", aprilTagProcessorBack.getPoseError().getHeading());
            telemetry.addData("drivetrain power", drive.getPoseEstimate());


            telemetry.update();
            intake.update();
            slide.update();
            outtake.update();
        }
        if(aprilTagProcessorBack.robotAligned())
            datalog.opModeStatus.set("BROKE FROM ALIGN");
        else
            datalog.opModeStatus.set("BROKE FROM TIME");
        datalog.writeLine();

        drive.setMotorPowers(0,0,0,0);
    }

    private void alignToApriltagStack() {
        frontCameraPortal.resumeLiveView();
        double currentTime = getRuntime();
        while(!isStopRequested()) {
            aprilTagProcessorFront.update();
            aprilTagProcessorFront.alignRobot(drive);

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
        intake.setMotorTargetPosition(-450);
        intake.update();
        intake.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.update();
        intake.setMotorPower(0.35);
        intake.update();
    }

    //OUTTAKE METHODS
    private void outtake() { outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.OUTSIDE); if(slide.getAvgPosition() >= -900) setSlideHeight(-900); }
    private void outtakeIn() { if(outtake.getOuttakePosition() == Outtake.OuttakePositions.OUTSIDE) setSlideHeight(-1000); outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.INSIDE); }
    private void outtakeTransfer() { if(outtake.getOuttakePosition() == Outtake.OuttakePositions.OUTSIDE) setSlideHeight(-1000); outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.TRANSFER); outtake.update(); }
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
                StartPosition.startPosition = StartPosition.StartPos.BLUE_CLOSE;
                AllianceHelper.alliance = AllianceHelper.Alliance.BLUE;
                break;
            }
            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                StartPosition.startPosition = StartPosition.StartPos.BLUE_FAR;
                AllianceHelper.alliance = AllianceHelper.Alliance.BLUE;
                break;
            }
            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                StartPosition.startPosition = StartPosition.StartPos.RED_FAR;
                AllianceHelper.alliance = AllianceHelper.Alliance.RED;
                break;
            }
            if (gamepad1.dpad_right || gamepad2.dpad_right) {
                StartPosition.startPosition = StartPosition.StartPos.RED_CLOSE;
                AllianceHelper.alliance = AllianceHelper.Alliance.RED;
                break;
            }
            telemetry.update();
        }
        while (!isStopRequested()) {
            telemetry.addLine("             [15118 AUTO INITIALIZED]");
            telemetry.addLine("-------------------------------------------------");
            telemetry.addLine(" Selected " + StartPosition.startPosition.toString() + " Starting Position.");
            telemetry.addLine();
            telemetry.addLine("Select Autonomous Path using DPAD Keys");
            telemetry.addData("     Inside      ", "(^)");
            telemetry.addData("     Outside     ", "(v)");

            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                Paths.path = Paths.Path.INSIDE;
                break;
            }
            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                Paths.path = Paths.Path.OUTSIDE;
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
        public Datalogger.GenericField xEstimate       = new Datalogger.GenericField("X Error");
        public Datalogger.GenericField yEstimate       = new Datalogger.GenericField("Y Error");
        public Datalogger.GenericField headingEstimate = new Datalogger.GenericField("Heading Error");
        public Datalogger.GenericField battery      = new Datalogger.GenericField("Battery");
        public Datalog(String name) {
            datalogger = new Datalogger.Builder()
                    .setFilename(name)
                    .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)
                    .setFields( opModeStatus, loopCounter, xEstimate, yEstimate, headingEstimate, xError, yError, headingError, battery )
                    .build();
        }
        public void writeLine()
        {
            datalogger.writeLine();
        }
    }
}