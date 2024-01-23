package org.firstinspires.ftc.teamcode.opmodes.auto;

import java.text.SimpleDateFormat;
import java.util.Date;
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
        if(startPosition == StartPosition.RED_CLOSE || startPosition == StartPosition.BLUE_CLOSE) {
            //TODO SPIKE PLACEMENT AND BACKBOARD
            if(path != Path.PLACEMENT) {
                //TODO CYCLE
            }
            //TODO PARK
        }
        else { //FAR AUTO
            //TODO SPIKE PLACEMENT
            if(path != Path.PLACEMENT) {
                //TODO BACKBOARD AND CYCLE
            }
            else {
                //TODO BACKBOARD
            }
            //TODO PARK
        }
    }

    private void autoLoop() {
        drive.update();
        intake.update();
        outtake.update();
        slide.update();
    }

    private void buildAuto() {
        switch(startPosition) {
            case RED_CLOSE:
                switch(purplePixelPath) {
                    case LEFT:
                        break;
                    case CENTER:
                        break;
                    case RIGHT:
                        break;
                }
                break;
            case RED_FAR:
                switch(purplePixelPath) {
                    case LEFT:
                        break;
                    case CENTER:
                        break;
                    case RIGHT:
                        break;
                }
                break;
            case BLUE_CLOSE:
                switch(purplePixelPath) {
                    case LEFT:
                        break;
                    case CENTER:
                        break;
                    case RIGHT:
                        break;
                }
                break;
            case BLUE_FAR:
                switch(purplePixelPath) {
                    case LEFT:
                        break;
                    case CENTER:
                        break;
                    case RIGHT:
                        break;
                }
                break;
        }
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

    private void detectPurplePath() { purplePixelPath = colorDetectionProcessor.getPosition(); }

    private void alignToApriltagBackboard() {
        double currentTime = getRuntime();
        while(!isStopRequested()) {
            aprilTagProcessorBack.update();
            aprilTagProcessorBack.alignRobotToBackboard(drive);

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

    //INTAKE METHODS
    private void intake() { intake.intakePower(1.0); intake.update(); }
    private void stopIntake() { intake.intakePower(0.0); intake.update(); }
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