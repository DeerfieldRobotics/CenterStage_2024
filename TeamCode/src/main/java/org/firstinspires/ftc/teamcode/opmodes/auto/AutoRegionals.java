package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Log;

import java.util.Calendar;
import java.util.Date;

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

    //TRAJECTORIES
    private TrajectorySequence backboardToSpike; // BACKBOARD TO SPIKE
    private TrajectorySequence backboardToWhite; //BACKBOARD TO STACK
    private TrajectorySequence spikeToWhite; //SPIKE TO STACK
    private TrajectorySequence spikeToPark; //SPIKE TO PARK
    private TrajectorySequence whiteToBackboard;// STACK BACK TO BACKBOARD
    private TrajectorySequence whiteToBackboardYellow; //STACK TO BACKBOARD WITH YELLOW
    private TrajectorySequence dropYellow; //DROP YELLOW
    private TrajectorySequence backboardToPark; //PARK AFTER SCORING STACK
    private enum CURRENT_TRAJECTORY {
        NONE,
        INIT,
        BACKBOARD_TO_SPIKE,
        BACKBOARD_TO_WHITE,
        SPIKE_TO_WHITE,
        SPIKE_TO_PARK,
        WHITE_TO_BACKBOARD,
        WHITE_TO_BACKBOARD_YELLOW,
        DROP_YELLOW,
        BACKBOARD_TO_PARK
    }
    private CURRENT_TRAJECTORY currentTrajectory = CURRENT_TRAJECTORY.NONE;
    //OTHER
    private int cycles = 0;
    private final String TAG = "AutoRegionals";

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
        telemetry.addData("Selected Path: ", Paths.path.toString());
        telemetry.addData("Detected Position: ", ColorDetectionProcessor.position.toString());
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

        TrajectorySequence init;

        if(StartPosition.startPosition == StartPosition.StartPos.RED_CLOSE || StartPosition.startPosition == StartPosition.StartPos.BLUE_CLOSE) {
            init = drive.trajectorySequenceBuilder(PoseHelper.initPose)
                    .setTangent(Math.toRadians(45 * PoseHelper.allianceAngleMultiplier))
                    .addTemporalMarker(this::outtake)
                    .splineToLinearHeading(PoseHelper.backboardPose, Math.toRadians(0))
                    .addTemporalMarker(() -> {
                        alignToApriltagBackboard();
                        if(!Double.isNaN(aprilTagProcessorBack.getPoseEstimate().getX()))
                            drive.setPoseEstimate(aprilTagProcessorBack.getPoseEstimate());
                        buildBackboardToSpike();
                        currentTrajectory = CURRENT_TRAJECTORY.BACKBOARD_TO_SPIKE;
                        drive.followTrajectorySequenceAsync(backboardToSpike); })
                    .build();
        }
        else { //FAR AUTO
            init = drive.trajectorySequenceBuilder(PoseHelper.initPose)
                    .setTangent(PoseHelper.initialFarTangent * PoseHelper.allianceAngleMultiplier)
                    .addTemporalMarker(() -> intake.setServoPosition(Intake.IntakePositions.INTAKE))
                    .splineToLinearHeading(PoseHelper.spikePose, PoseHelper.spikePose.getHeading())
                    .addTemporalMarker(() -> currentTrajectory = CURRENT_TRAJECTORY.SPIKE_TO_WHITE)
                    .addTemporalMarker(() -> drive.followTrajectorySequenceAsync(spikeToWhite))
                    .build();
            spikeToWhite = drive.trajectorySequenceBuilder(PoseHelper.spikePose)
                    .addTemporalMarker(this::outtakePurple)
                    .back(PoseHelper.purpleBackDistanceFar)
                    .addTemporalMarker(() -> intake.setServoPosition(Intake.IntakePositions.FIVE))
                    .setTangent(Math.toRadians(PoseHelper.toWhiteStackTangentFar))
                    .splineToLinearHeading(PoseHelper.stackPose, Math.toRadians(PoseHelper.toWhiteStackTangentFar))
                    .addTemporalMarker(() -> currentTrajectory = CURRENT_TRAJECTORY.WHITE_TO_BACKBOARD_YELLOW)
                    .addTemporalMarker(() -> drive.followTrajectorySequenceAsync(whiteToBackboardYellow))
                    .build();
            whiteToBackboardYellow = drive.trajectorySequenceBuilder(PoseHelper.stackPose)
                    .addTemporalMarker(this::intake)
                    .forward(2)
                    .back(2)
                    .addTemporalMarker(this::stopIntake)
                    .splineToLinearHeading(PoseHelper.wingTruss, Math.toRadians(0))
                    .addTemporalMarker(() -> intake.setBoosterServoPower(0.0))
                    .splineToLinearHeading(PoseHelper.boardTruss, Math.toRadians(0))
                    .addTemporalMarker(this::outtake)
                    .addTemporalMarker(() -> setSlideHeight(-1050))
                    .splineToLinearHeading(PoseHelper.backboardPose, Math.toRadians(30.0 * PoseHelper.allianceAngleMultiplier))
                    .addTemporalMarker(() -> {
                        alignToApriltagBackboard();
                        if(!Double.isNaN(aprilTagProcessorBack.getPoseEstimate().getX()))
                            drive.setPoseEstimate(aprilTagProcessorBack.getPoseEstimate());
                        buildDropYellow();
                        currentTrajectory = CURRENT_TRAJECTORY.DROP_YELLOW;
                        drive.followTrajectorySequenceAsync(dropYellow);
                    })
                    .build();
        }

        currentTrajectory = CURRENT_TRAJECTORY.INIT;
        drive.followTrajectorySequenceAsync(init);
    }

    private void buildSpikeToPark() {
        spikeToPark = drive.trajectorySequenceBuilder(PoseHelper.spikePose)
                .splineToLinearHeading(PoseHelper.parkPose, Math.toRadians(180.0))
                .addTemporalMarker(() -> {
                    outtakeTransfer(); //transfer just for fun
                    transfer();
                    currentTrajectory = CURRENT_TRAJECTORY.NONE;
                })
                .build();
    }

    private void buildSpikeToWhite() {
        spikeToWhite = drive.trajectorySequenceBuilder(PoseHelper.spikePose)
                .setTangent(Math.toRadians(Paths.path == Paths.Path.OUTSIDE ? 315.0 : 45.0 * PoseHelper.allianceAngleMultiplier))
                .splineToLinearHeading(PoseHelper.boardTruss, Math.toRadians(180.0))
                .addTemporalMarker(() -> intake.setServoPosition(Intake.IntakePositions.FOUR))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(PoseHelper.wingTruss, Math.toRadians(180.0))
                .splineToLinearHeading(PoseHelper.stackPose, Math.toRadians(180.0))
                .addTemporalMarker(() -> {
                    buildWhiteToBackboard();
                    currentTrajectory = CURRENT_TRAJECTORY.WHITE_TO_BACKBOARD;
                    drive.followTrajectorySequenceAsync(whiteToBackboard);
                })
                .build();
    }

    private void buildBackboardToSpike() {
        backboardToSpike = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
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
                    if(Paths.path != Paths.Path.PLACEMENT) {
                        buildSpikeToWhite();
                        currentTrajectory = CURRENT_TRAJECTORY.SPIKE_TO_WHITE;
                        drive.followTrajectorySequenceAsync(spikeToWhite);
                    }
                    else {
                        buildSpikeToPark();
                        currentTrajectory = CURRENT_TRAJECTORY.SPIKE_TO_PARK;
                        drive.followTrajectorySequenceAsync(spikeToPark);
                    }
                })
                .build();
    }

    private void buildDropYellow() {
        dropYellow = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .waitSeconds(0.1)
                .back(5)
                .addTemporalMarker(this::drop)
                .addTemporalMarker(()->setSlideHeight(-1200))
                .forward(5)
                .addTemporalMarker(this::outtakeTransfer)
                .waitSeconds(0.8)
                .addTemporalMarker(this::transfer)
                .waitSeconds(1.3)
                .addTemporalMarker(this::outtake)
                .addTemporalMarker(() -> setSlideHeight(-1400))
                .addTemporalMarker(() -> {
                    //DEFAULTS TO RIGHT SIDE IF POSSIBLE
                    if(PoseHelper.backboardPose == PoseHelper.backboardRightRed || PoseHelper.backboardPose == PoseHelper.backboardLeftBlue) {
                        if (AllianceHelper.alliance == AllianceHelper.Alliance.RED)
                            PoseHelper.backboardPose = PoseHelper.backboardCenterRed;
                        else
                            PoseHelper.backboardPose = PoseHelper.backboardCenterBlue;
                    }
                    else {
                        if (AllianceHelper.alliance == AllianceHelper.Alliance.RED)
                            PoseHelper.backboardPose = PoseHelper.backboardRightRed;
                        else
                            PoseHelper.backboardPose = PoseHelper.backboardLeftBlue;
                    }
                    aprilTagProcessorBack.setPIDCoefficients(.042, .038, 0.0, .03, .02, 0, 0.82, 0.02, 0.0);
                    alignToApriltagBackboard();
                    if(Double.isNaN(aprilTagProcessorBack.getPoseEstimate().getX()))
                        drive.setPoseEstimate(aprilTagProcessorBack.getPoseEstimate());
                    if(Paths.path != Paths.Path.PLACEMENT) {
                        buildBackboardToWhite();
                        currentTrajectory = CURRENT_TRAJECTORY.BACKBOARD_TO_WHITE;
                        drive.followTrajectorySequenceAsync(backboardToWhite);
                    }
                    else {
                        buildBackboardToPark();
                        currentTrajectory = CURRENT_TRAJECTORY.BACKBOARD_TO_PARK;
                        drive.followTrajectorySequenceAsync(backboardToPark);
                    }
                })
                .build();

    }

    private void buildBackboardToWhite() {
        backboardToWhite = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .waitSeconds(0.1)
                .back(6)
                .addTemporalMarker(this::drop)
                .waitSeconds(0.5)
                .addTemporalMarker(this::outtakeIn)
                .addTemporalMarker(() -> setSlideHeight(-1200))
                .setTangent(Math.toRadians(180.0))
                .splineToConstantHeading(PoseHelper.boardTruss.vec(), Math.toRadians(180.0))
                .addTemporalMarker(() -> {
                    if(Paths.path == Paths.Path.OUTSIDE)
                        intake.setServoPosition(Intake.IntakePositions.THREE);
                    else {
                        if(PoseHelper.backboardPose == PoseHelper.backboardLeftRed || PoseHelper.backboardPose == PoseHelper.backboardRightBlue) {
                            if (AllianceHelper.alliance == AllianceHelper.Alliance.RED)
                                PoseHelper.backboardPose = PoseHelper.backboardCenterRed;
                            else
                                PoseHelper.backboardPose = PoseHelper.backboardCenterBlue;
                        }
                        else {
                            if (AllianceHelper.alliance == AllianceHelper.Alliance.RED)
                                PoseHelper.backboardPose = PoseHelper.backboardLeftRed;
                            else
                                PoseHelper.backboardPose = PoseHelper.backboardRightBlue;
                        }
                        if (cycles == 0)
                            intake.setServoPosition(Intake.IntakePositions.TWO);
                        else
                            intake.setServoPosition(Intake.IntakePositions.TWO);
                    }
                    cycles++;
                    intake.update();
                })
                .splineToConstantHeading(PoseHelper.wingTruss.vec(), Math.toRadians(180.0))
                .splineToConstantHeading(PoseHelper.stackPose.vec(), Math.toRadians(180.0))
                .addTemporalMarker(() -> {
                    buildWhiteToBackboard();
                    currentTrajectory = CURRENT_TRAJECTORY.WHITE_TO_BACKBOARD;
                    drive.followTrajectorySequenceAsync(whiteToBackboard);
                })
                .build();
    }

    private void buildWhiteToBackboard() {
        whiteToBackboard = drive.trajectorySequenceBuilder(PoseHelper.stackPose)
                .addTemporalMarker(this::intake)
                .addTemporalMarker(this::outtakeTransfer)
                .forward(2)
                .back(2)
                .addTemporalMarker(this::stopIntake)
                .addTemporalMarker(this::transfer)
                .splineToLinearHeading(PoseHelper.wingTruss, Math.toRadians(0))
                .splineToLinearHeading(PoseHelper.boardTruss, Math.toRadians(0))
                .addTemporalMarker(this::outtake)
                .addTemporalMarker(() -> setSlideHeight(-1500))
                .splineToLinearHeading(PoseHelper.backboardPose, Math.toRadians(Paths.path == Paths.Path.OUTSIDE ? 30.0 : -30.0 * PoseHelper.allianceAngleMultiplier))
                .addTemporalMarker(() -> {
                    alignToApriltagBackboard();
                    if(Double.isNaN(aprilTagProcessorBack.getPoseEstimate().getX()))
                        drive.setPoseEstimate(aprilTagProcessorBack.getPoseEstimate());
                    if(cycles == 0) {
                        buildBackboardToWhite();
                        currentTrajectory = CURRENT_TRAJECTORY.BACKBOARD_TO_WHITE;
                        drive.followTrajectorySequenceAsync(backboardToWhite);
                    }
                    else if (cycles == 1) {
                        buildBackboardToPark();
                        currentTrajectory = CURRENT_TRAJECTORY.BACKBOARD_TO_PARK;
                        drive.followTrajectorySequenceAsync(backboardToPark);
                    }
                })
                .build();
    }

    private void buildBackboardToPark() {
        backboardToPark = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setTangent(Math.toRadians(Paths.path == Paths.Path.OUTSIDE ? -150.0 : 150.0 * PoseHelper.allianceAngleMultiplier))
                .splineToLinearHeading(PoseHelper.parkPose, Math.toRadians(180.0))
                .addTemporalMarker(() -> {
                    outtakeTransfer(); //transfer just for fun
                    transfer();
                    currentTrajectory = CURRENT_TRAJECTORY.NONE;
                })
                .build();
    }

    private void autoLoop() {
        drive.update();
        intake.update();
        outtake.update();
        slide.update();
        Log.d(TAG, "drivePose" + drive.getPoseEstimate());
        Log.d(TAG, "intakePosition" + intake.getServoPosition());
        Log.d(TAG, "outtakePosition" + outtake.getOuttakePosition());
        Log.d(TAG, "outtakeTargetPosition" + outtake.getOuttakeProcedureTarget());
        Log.d(TAG, "slidePosition" + slide.getAvgPosition());
        Log.d(TAG, "slideTargetPosition" + slide.getTargetPosition());
        Log.d(TAG, "currentTrajectory" + currentTrajectory);
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

            Log.d(TAG, "apriltag error pose" + aprilTagProcessorBack.getPoseError());
            Log.d(TAG, "apriltag pose" + aprilTagProcessorBack.getPoseEstimate());


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
    private void outtake() { outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.OUTSIDE); if(slide.getAvgPosition() >= -950) setSlideHeight(-950); }
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
            telemetry.addLine("Select Autonomous Path using Shape Buttons");
            telemetry.addData("     Inside      ", "(Triangle)");
            telemetry.addData("     Outside     ", "(Cross)");
            telemetry.addData("     Placement   ", "(Circle)");

            if (gamepad1.triangle || gamepad2.triangle) {
                Paths.path = Paths.Path.INSIDE;
                break;
            }
            if (gamepad1.cross || gamepad2.cross) {
                Paths.path = Paths.Path.OUTSIDE;
                break;
            }
            if (gamepad1.circle || gamepad2.circle) {
                Paths.path = Paths.Path.PLACEMENT;
                break;
            }
            telemetry.update();
        }
        telemetry.clear();
    }
}