package org.firstinspires.ftc.teamcode.opmodes.auto;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.CogchampDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
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
        BACKBOARD_TO_PARK,
        WHITE_TO_PARK,
    }
    private CURRENT_TRAJECTORY currentTrajectory = CURRENT_TRAJECTORY.NONE;

    //OTHER
    private int cycles = 0;
    private final String TAG = "AutoRegionals";
    private boolean verbose = false;
    private double lastRunTime = 0.0;

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
        telemetry.addData("Front Camera State: ", frontCameraPortal.getCameraState().toString());
        telemetry.addData("Back Camera State: ", backCameraPortal.getCameraState().toString());
        telemetry.update();

        if(backCameraPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            backCameraPortal.stopStreaming();
        }
    }

    private void startAuto() {
        resetRuntime();
        frontCameraPortal.stopLiveView();
        backCameraPortal.resumeStreaming();
        PoseHelper.buildAuto();
        drive.setPoseEstimate(PoseHelper.initPose);

        TrajectorySequence init;

        if(StartPosition.startPosition == StartPosition.StartPos.RED_CLOSE || StartPosition.startPosition == StartPosition.StartPos.BLUE_CLOSE) {
            init = drive.trajectorySequenceBuilder(PoseHelper.initPose)
                    .setVelConstraint(PoseHelper.toBackboardVelocityConstraint)
                    .setTangent(Math.toRadians(45 * PoseHelper.allianceAngleMultiplier))
                    .addTemporalMarker(this::outtake)
                    .splineToLinearHeading(PoseHelper.backboardPose, Math.toRadians(0))
                    .addTemporalMarker(() -> {
                        alignToApriltagBackboard();
                        apriltagToDrivePose();
                        buildBackboardToSpike();
                        logTrajectory(CURRENT_TRAJECTORY.BACKBOARD_TO_SPIKE);
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
                    .splineToLinearHeading(AllianceHelper.alliance == AllianceHelper.Alliance.RED ? PoseHelper.backboardCenterRed : PoseHelper.backboardCenterBlue, Math.toRadians(Paths.path == Paths.Path.OUTSIDE ? 30.0 : -30.0 * PoseHelper.allianceAngleMultiplier))
                    .addTemporalMarker(() -> {
                        if(ColorDetectionProcessor.position != ColorDetectionProcessor.StartingPosition.CENTER) {
                            apriltagTuckerCarlson();
                        }
                        alignToApriltagBackboard();
                        apriltagToDrivePose();
                        buildDropYellow();
                        logTrajectory(CURRENT_TRAJECTORY.DROP_YELLOW);
                        drive.followTrajectorySequenceAsync(dropYellow);
                    })
                    .build();
        }

        logTrajectory(CURRENT_TRAJECTORY.INIT);
        drive.followTrajectorySequenceAsync(init);
    }

    private void buildBackboardToSpike() {
        backboardToSpike = drive.trajectorySequenceBuilder(PoseHelper.currentPose)
                .setVelConstraint(PoseHelper.toPurpleVelocityConstraint)
                .back(PoseHelper.backboardBackup)
                .addTemporalMarker(this::drop)
                .waitSeconds(0.2)
                .addTemporalMarker(() -> setSlideHeight(-1100))
                .waitSeconds(0.2)
                .forward(PoseHelper.backboardBackup)
                .addTemporalMarker(this::outtakeIn)
                .addTemporalMarker(() -> intake.setServoPosition(Intake.IntakePositions.INTAKE))
                .setTangent(Math.toRadians(180.0))
                .splineToLinearHeading(PoseHelper.spikePose, Math.toRadians(180.0))
                .back(4)
                .addTemporalMarker(this::outtakePurple)
                .addTemporalMarker(() -> {
                    if(Paths.path != Paths.Path.PLACEMENT) {
                        buildSpikeToWhite();
                        logTrajectory(CURRENT_TRAJECTORY.SPIKE_TO_WHITE);
                        drive.followTrajectorySequenceAsync(spikeToWhite);
                    }
                    else {
                        buildSpikeToPark();
                        logTrajectory(CURRENT_TRAJECTORY.SPIKE_TO_PARK);
                        drive.followTrajectorySequenceAsync(spikeToPark);
                    }
                })
                .build();
    }

    private void buildSpikeToWhite() {
        spikeToWhite = drive.trajectorySequenceBuilder(PoseHelper.spikePose)
                .waitSeconds(0.1)
                .back(4)
                .setTangent(Math.toRadians(
                        ((ColorDetectionProcessor.position == ColorDetectionProcessor.StartingPosition.RIGHT && AllianceHelper.alliance == AllianceHelper.Alliance.RED) ||
                                (ColorDetectionProcessor.position == ColorDetectionProcessor.StartingPosition.LEFT && AllianceHelper.alliance == AllianceHelper.Alliance.BLUE))
                                ? 45 * PoseHelper.allianceAngleMultiplier : 0))
                .splineToConstantHeading(PoseHelper.aprilTruss.vec(), Math.toRadians(180.0))
                .addTemporalMarker(() -> intake.setServoPosition(Intake.IntakePositions.FOUR))
                .waitSeconds(0.5)
                .addTemporalMarker(this::aprilTagRelocalize)
                .waitSeconds(0.3)
                .setTangent(Math.toRadians(140*PoseHelper.allianceAngleMultiplier))
                .splineToConstantHeading(PoseHelper.wingTruss.vec(), Math.toRadians(180.0))
                .addTemporalMarker(this::intake)
                .splineToConstantHeading(PoseHelper.stackPose.vec(), Math.toRadians(180.0))
                .addTemporalMarker(() -> {
                    buildWhiteToBackboard();
                    logTrajectory(CURRENT_TRAJECTORY.WHITE_TO_BACKBOARD);
                    drive.followTrajectorySequenceAsync(whiteToBackboard);
                })
                .build();
    }

    private void buildSpikeToPark() {
        spikeToPark = drive.trajectorySequenceBuilder(PoseHelper.spikePose)
                .back(5)
                .setTangent(0)
                .strafeLeft(24*PoseHelper.allianceAngleMultiplier)
                .splineToLinearHeading(PoseHelper.parkPose, Math.toRadians(180.0))
                .addTemporalMarker(() -> {
                    outtakeTransfer(); //transfer just for fun
                    transfer();
                    logTrajectory(CURRENT_TRAJECTORY.NONE);
                })
                .build();
    }

    private void buildDropYellow() {
        dropYellow = drive.trajectorySequenceBuilder(PoseHelper.currentPose)
                .back(PoseHelper.backboardBackup)
                .addTemporalMarker(this::drop)
                .waitSeconds(0.4)
                .addTemporalMarker(()->setSlideHeight(-1200))
                .forward(PoseHelper.backboardBackup)
                .addTemporalMarker(this::outtakeTransfer)
                .waitSeconds(0.8)
                .addTemporalMarker(this::transfer)
                .waitSeconds(1.6)
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
                    apriltagTuckerCarlson();
                    alignToApriltagBackboard();
                    apriltagToDrivePose();
                    if(Paths.path != Paths.Path.PLACEMENT) {
                        buildBackboardToWhite();
                        logTrajectory(CURRENT_TRAJECTORY.BACKBOARD_TO_WHITE);
                        drive.followTrajectorySequenceAsync(backboardToWhite);
                    }
                    else {
                        buildBackboardToPark();
                        logTrajectory(CURRENT_TRAJECTORY.BACKBOARD_TO_PARK);
                        drive.followTrajectorySequenceAsync(backboardToPark);
                    }
                })
                .build();
    }

    private void buildBackboardToWhite() {
        while(!isStopRequested()) {
            try {
                backboardToWhite = drive.trajectorySequenceBuilder(PoseHelper.currentPose)
                        .back(PoseHelper.backboardBackup)
                        .addTemporalMarker(this::drop)
                        .waitSeconds(0.4)
                        .addTemporalMarker(this::outtakeIn)
                        .addTemporalMarker(() -> setSlideHeight(-1200))
                        .splineToConstantHeading(PoseHelper.aprilTruss.vec(), Math.toRadians(180.0))
                        .addTemporalMarker(this::aprilTagRelocalize)
                        .addTemporalMarker(() -> {
                            if (Paths.path == Paths.Path.OUTSIDE)
                                intake.setServoPosition(Intake.IntakePositions.THREE);
                            else {
                                if (cycles == 0)
                                    intake.setServoPosition(Intake.IntakePositions.TWO);
                                else
                                    intake.setServoPosition(Intake.IntakePositions.TWO);
                            }
                            cycles++;
                            intake.update();
                        })
                        .setTangent(Math.toRadians(Paths.path == Paths.Path.INSIDE ? 140 : -150 ) * PoseHelper.allianceAngleMultiplier)
                        .splineToConstantHeading(PoseHelper.wingTruss.vec(), Math.toRadians(180.0))
                        .addTemporalMarker(this::intake)
                        .splineToConstantHeading(PoseHelper.stackPose.plus(PoseHelper.stackOffset).vec(), Math.toRadians(Paths.path == Paths.Path.INSIDE ? 180 : 120 * PoseHelper.allianceAngleMultiplier))
                        .addTemporalMarker(() -> {
                            if((cycles == 0 && Paths.path == Paths.Path.INSIDE) || ((cycles == 0 || cycles == 1) && Paths.path == Paths.Path.OUTSIDE)) {
                                buildWhiteToBackboard();
                                logTrajectory(CURRENT_TRAJECTORY.WHITE_TO_BACKBOARD);
                                drive.followTrajectorySequenceAsync(whiteToBackboard);
                            }
                            else {
                                buildWhiteToPark();
                                logTrajectory(CURRENT_TRAJECTORY.WHITE_TO_PARK);
                                drive.followTrajectorySequenceAsync(whiteToBackboard);
                            }
                        })
                        .build();
                break;
            } catch (Exception e) {
                Log.wtf(TAG, "Error building backboard to white: " + e);
                Log.d(TAG, "Drive Pose: "+drive.getPoseEstimate());
                Log.d(TAG, "Current Pose: "+PoseHelper.currentPose);
            }
        }
    }

    private void buildWhiteToBackboard() {
        whiteToBackboard = drive.trajectorySequenceBuilder(PoseHelper.stackPose)
                .setVelConstraint(cycles == 1 && Paths.path == Paths.Path.INSIDE || cycles == 2 && Paths.path == Paths.Path.OUTSIDE ? PoseHelper.blastVelocityConstraint : PoseHelper.toBackboardVelocityConstraint)
                .setAccelConstraint(cycles == 1 && Paths.path == Paths.Path.INSIDE || cycles == 2 && Paths.path == Paths.Path.OUTSIDE ? PoseHelper.blastAccelerationConstraint : PoseHelper.toBackboardAccelerationConstraint)
                .addTemporalMarker(this::outtakeTransfer)
                .forward(1.5)
                .back(3)
                .addTemporalMarker(this::stopIntake)
                .addTemporalMarker(this::transfer)
                .splineToLinearHeading(PoseHelper.wingTruss, Math.toRadians(0))
                .splineToLinearHeading(PoseHelper.boardTruss, Math.toRadians(0))
                .addTemporalMarker(this::outtake)
                .addTemporalMarker(() -> setSlideHeight(-1500))
                .splineToLinearHeading(AllianceHelper.alliance == AllianceHelper.Alliance.RED ? PoseHelper.backboardCenterRed : PoseHelper.backboardCenterBlue, Math.toRadians(Paths.path == Paths.Path.OUTSIDE ? 30.0 : -30.0 * PoseHelper.allianceAngleMultiplier))
                .addTemporalMarker(() -> {
                    if(Paths.path == Paths.Path.INSIDE && cycles == 0) {
                        if (PoseHelper.backboardPose == PoseHelper.backboardLeftRed || PoseHelper.backboardPose == PoseHelper.backboardRightBlue) {
                            if (AllianceHelper.alliance == AllianceHelper.Alliance.RED)
                                PoseHelper.backboardPose = PoseHelper.backboardCenterRed;
                            else
                                PoseHelper.backboardPose = PoseHelper.backboardCenterBlue;
                        } else {
                            if (AllianceHelper.alliance == AllianceHelper.Alliance.RED)
                                PoseHelper.backboardPose = PoseHelper.backboardLeftRed;
                            else
                                PoseHelper.backboardPose = PoseHelper.backboardRightBlue;
                        }
                    }

                    apriltagTuckerCarlson();

                    alignToApriltagBackboard();

                    apriltagToDrivePose();

                    if((cycles == 0 && Paths.path == Paths.Path.INSIDE) || (cycles == 0 || cycles == 1) && Paths.path == Paths.Path.OUTSIDE) {
                        buildBackboardToWhite();
                        logTrajectory(CURRENT_TRAJECTORY.BACKBOARD_TO_WHITE);
                        drive.followTrajectorySequenceAsync(backboardToWhite);
                    }
                    else {
                        backCameraPortal.close();
                        frontCameraPortal.close();

                        buildBackboardToPark();
                        logTrajectory(CURRENT_TRAJECTORY.BACKBOARD_TO_PARK);
                        drive.followTrajectorySequenceAsync(backboardToPark);
                    }
                })
                .build();
    }

    public void buildWhiteToPark() {
        whiteToBackboard = drive.trajectorySequenceBuilder(PoseHelper.stackPose)
                .setVelConstraint(PoseHelper.blastVelocityConstraint)
                .setAccelConstraint(PoseHelper.blastAccelerationConstraint)
                .addTemporalMarker(() -> {
                    backCameraPortal.close();
                    frontCameraPortal.close();
                })
                .addTemporalMarker(this::outtakeTransfer)
                .forward(1.5)
                .back(1.5)
                .addTemporalMarker(this::stopIntake)
                .addTemporalMarker(this::transfer)
                .splineToLinearHeading(PoseHelper.wingTruss, Math.toRadians(0))
                .splineToLinearHeading(PoseHelper.boardTruss, Math.toRadians(0))
                .splineToLinearHeading(PoseHelper.parkPose, Math.toRadians(0))
                .build();

    }

    private void buildBackboardToPark() {
        backboardToPark = drive.trajectorySequenceBuilder(PoseHelper.currentPose)
                .back(PoseHelper.backboardBackup)
                .addTemporalMarker(this::drop)
                .addTemporalMarker(() -> setSlideHeight(-1600))
                .waitSeconds(.4)
                .addTemporalMarker(this::outtakeIn)
                .forward(5)
                .strafeRight(8.0 * PoseHelper.allianceAngleMultiplier * (Paths.path == Paths.Path.INSIDE ? 1.0 : -1.0))
                .addTemporalMarker(() -> {
                    outtakeTransfer(); //transfer just for fun
                    transfer();
                    logTrajectory(CURRENT_TRAJECTORY.NONE);
                })
                .build();
    }

    private void autoLoop() {
        drive.update();
        intake.update();
        outtake.update();
        slide.update();
        if(verbose) {
            Log.d(TAG, "drivePose" + drive.getPoseEstimate());
            Log.d(TAG, "intakePosition" + intake.getServoPosition());
            Log.d(TAG, "outtakePosition" + outtake.getOuttakePosition());
            Log.d(TAG, "outtakeTargetPosition" + outtake.getOuttakeProcedureTarget());
            Log.d(TAG, "slidePosition" + slide.getAvgPosition());
            Log.d(TAG, "slideTargetPosition" + slide.getTargetPosition());
            Log.d(TAG, "currentTrajectory" + currentTrajectory);
        }
    }

    private void logTrajectory(CURRENT_TRAJECTORY trajectory) {
        currentTrajectory = trajectory;
        Log.d(TAG, "last trajectory time " + (getRuntime() - lastRunTime));
        Log.d(TAG, "currentTrajectory " + currentTrajectory);
        Log.d(TAG, "drivePose " + drive.getPoseEstimate());
        Log.d(TAG, "runtime " + getRuntime());
        lastRunTime = getRuntime();
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

    private void aprilTagRelocalize() {
        aprilTagProcessorBack.update();
        Log.d("POSE", "AprilTag Pose: " + aprilTagProcessorBack.getPoseEstimate());
        Log.d("POSE", "Drive Pose" + drive.getPoseEstimate());
        if(!Double.isNaN(aprilTagProcessorBack.getPoseEstimate().getX())) {
            drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), aprilTagProcessorBack.getPoseEstimate().getY(), aprilTagProcessorBack.getPoseEstimate().getHeading()));
        }
        Log.d("POSE", "Set Pose" + drive.getPoseEstimate());
    }

    private void alignToApriltagBackboard() {
        aprilTagProcessorBack.setTargetPose(PoseHelper.backboardPose);
        double currentTime = getRuntime();
        while(!isStopRequested()) {
            aprilTagProcessorBack.update();

            if(verbose) {
                Log.d(TAG, "apriltag error pose" + aprilTagProcessorBack.getPoseError());
                Log.d(TAG, "apriltag pose" + aprilTagProcessorBack.getPoseEstimate());
            }

            if(getRuntime()-currentTime > .75) break;

            aprilTagProcessorBack.alignRobot(drive);

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

            intake.update();
            slide.update();
            outtake.update();

            if(getRuntime()-currentTime > 2 || aprilTagProcessorBack.robotAligned()) break;
        }
    }

    private void apriltagTuckerCarlson() {
        aprilTagProcessorBack.setPIDCoefficients(.042, .038, 0.0, .030, .012, 0, 0.82, 0.02, 0.0);
    }

    private void apriltagToDrivePose() {
        Log.d("POSE", "AprilTag Pose: " + aprilTagProcessorBack.getPoseEstimate());
        Log.d("POSE", "Drive Pose" + drive.getPoseEstimate());
        if(!Double.isNaN(aprilTagProcessorBack.getPoseEstimate().getX())) {
            drive.setPoseEstimate(aprilTagProcessorBack.getPoseEstimate());
        }
        PoseHelper.currentPose = drive.getPoseEstimate();
        Log.d("POSE", "Set Pose" + drive.getPoseEstimate());
    }

    //INTAKE METHODS
    private void intake() { intake.intakePower(1.0); intake.update(); }
    private void stopIntake() {
        intake.intakePower(0.0);
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
            telemetry.addLine("Select Autonomous Path using Shape Buttons");
            telemetry.addData("     Inside      ", "(Triangle)");
            telemetry.addData("     Outside     ", "(Cross)");
            telemetry.addData("     Placement   ", "(Circle)");
            telemetry.addLine();
            telemetry.addLine("Select Logging Level using Bumper Buttons");
            telemetry.addData(verbose ? "-----verbose-----" : "     verbose     " , "(L2)");
            telemetry.addData(!verbose ? "-----normal------" : "     normal      " , "(R2)");

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

            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                verbose = true;
            }
            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                verbose = false;
            }

            telemetry.update();
        }
        telemetry.clear();
    }
}