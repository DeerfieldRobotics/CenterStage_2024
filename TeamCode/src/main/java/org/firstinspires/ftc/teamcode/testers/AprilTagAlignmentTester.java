package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.CogchampDrive;
import org.firstinspires.ftc.teamcode.utils.Other.Datalogger;
import org.firstinspires.ftc.teamcode.utils.detection.AllianceHelper;
import org.firstinspires.ftc.teamcode.utils.detection.AprilTagAlignment;

@TeleOp(name = "AprilTagTester")
@Config
public class AprilTagAlignmentTester extends LinearOpMode {
    private CogchampDrive drivetrain;
    private WebcamName webcam;
    private AprilTagAlignment aprilTagAlignment;
    private Datalog datalog;
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

    @Override
    public void runOpMode() throws InterruptedException {
        datalog = new Datalog("AprilTagAlignmentTester");

        xPID = new PIDController(xP, xI, xD);
        yPID = new PIDController(yP, yI, yD);
        headingPID = new PIDController(headingP, headingI, headingD);

        drivetrain = new CogchampDrive(hardwareMap);
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        aprilTagAlignment = new AprilTagAlignment(webcam, 0.0, 12.0, 0.0, AllianceHelper.Alliance.RED, (xPID), (yPID), (headingPID));

        waitForStart();

        while (opModeIsActive()) {
//            xPID.setPID(xP, xI, xD);
//            yPID.setPID(yP, yI, yD);
//            headingPID.setPID(headingP, headingI, headingD);
            aprilTagAlignment.update();

            if(gamepad1.left_bumper) {
                aprilTagAlignment.alignRobotToBackboard(drivetrain);
                aprilTagAlignment.setTargetX(aprilTagAlignment.getTargetX()+0.01*gamepad1.left_stick_x);
            }
            else
                driveNormal();

            telemetry.addData("targetTagID", aprilTagAlignment.getTargetTagID());
            telemetry.addData("targetFound", aprilTagAlignment.getTargetFound());
            telemetry.addData("targetX", aprilTagAlignment.getTargetX());
            telemetry.addData("currentX", aprilTagAlignment.getCurrentX());
            telemetry.addData("currentY", aprilTagAlignment.getCurrentY());
            telemetry.addData("currentHeading", aprilTagAlignment.getCurrentHeading());
            telemetry.addData("x error","%5.1f inches", aprilTagAlignment.getXError());
            telemetry.addData("y error","%5.1f inches", aprilTagAlignment.getYError());
            telemetry.addData("heading error","%3.0f degrees", aprilTagAlignment.getHeadingError());
            telemetry.addData("xPower", aprilTagAlignment.getXPower());
            telemetry.addData("yPower", aprilTagAlignment.getYPower());
            telemetry.addData("headingPower", aprilTagAlignment.getHeadingPower());
            telemetry.addData("forward", aprilTagAlignment.getForward());
            telemetry.addData("strafe", aprilTagAlignment.getStrafe());
            telemetry.addData("turn", aprilTagAlignment.getTurn());
            telemetry.update();

            datalog.xError.set(aprilTagAlignment.getXError());
            datalog.yError.set(aprilTagAlignment.getYError());
            datalog.headingError.set(aprilTagAlignment.getHeadingError());
            datalog.writeLine();
        }
    }
    private void driveNormal() {
        double speedMult = .7+0.3 * gamepad1.right_trigger-0.3*gamepad1.left_trigger;

        gamepad1.rumble(gamepad1.left_trigger>0.5?(gamepad1.left_trigger-0.5)/.4:0.0,gamepad1.right_trigger>0.4?(gamepad1.right_trigger-0.4)/0.8:0.0,50);

        double forwardMult = 1;
        double turnMult = .75;
        double strafeMult = 1;

        double forward = gamepad1.left_stick_y * forwardMult * speedMult;
        double turn = -gamepad1.right_stick_x * turnMult * speedMult;
        double strafe = -gamepad1.left_stick_x * strafeMult * speedMult;

        drivetrain.setWeightedDrivePower(new Pose2d(forward, strafe, turn));
    }
    public static class Datalog
    {
        // The underlying datalogger object - it cares only about an array of loggable fields
        private final Datalogger datalogger;

        // These are all of the fields that we want in the datalog.
        // Note that order here is NOT important. The order is important in the setFields() call below
        public Datalogger.GenericField xError       = new Datalogger.GenericField("X Error");
        public Datalogger.GenericField yError       = new Datalogger.GenericField("Y Error");
        public Datalogger.GenericField headingError = new Datalogger.GenericField("Heading Error");

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
                            xError,
                            yError,
                            headingError
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

