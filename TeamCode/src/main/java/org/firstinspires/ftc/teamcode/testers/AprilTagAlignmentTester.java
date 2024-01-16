package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.Other.Datalogger;
import org.firstinspires.ftc.teamcode.utils.detection.AprilTagAlignment;
import org.firstinspires.ftc.teamcode.utils.hardware.Drivetrain;

import java.util.Arrays;
import java.util.Collections;

@Config
@TeleOp(name = "AprilTagTester")
public class AprilTagAlignmentTester extends LinearOpMode {
    private Drivetrain drivetrain;
    private WebcamName webcam;
    private AprilTagAlignment aprilTagAlignment;
    private Datalog datalog;
    private boolean dpadLeftToggle = false;
    private boolean dpadRightToggle = false;
    public static PIDController xPID = new PIDController(0.0174, 0.0, 0.0);
    public static PIDController yPID = new PIDController(0.0174, 0.0, 0.0);
    public static PIDController headingPID = new PIDController(0.0174, 0.0, 0.0);

    @Override
    public void runOpMode() throws InterruptedException {
        datalog = new Datalog("AprilTagAlignmentTester");

        drivetrain = new Drivetrain(hardwareMap);
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        aprilTagAlignment = new AprilTagAlignment(webcam, drivetrain, 0.0, 12.0, 0.0, (xPID), (yPID), (headingPID));

        waitForStart();

        while (opModeIsActive()) {
            aprilTagAlignment.update();

            if(gamepad1.left_bumper) {
                aprilTagAlignment.alignRobot();
            }
            else
                driveNormal();

            if(gamepad1.dpad_left && !dpadLeftToggle && aprilTagAlignment.getTargetTagID() > 1) {
                aprilTagAlignment.setTargetTagID(aprilTagAlignment.getTargetTagID()-1);
                dpadLeftToggle = true;
            }
            else if(!gamepad1.dpad_left && dpadLeftToggle)
                dpadLeftToggle = false;

            if(gamepad1.dpad_right && !dpadRightToggle && aprilTagAlignment.getTargetTagID() < 6) {
                aprilTagAlignment.setTargetTagID(aprilTagAlignment.getTargetTagID()+1);
                dpadRightToggle = true;
            }
            else if(!gamepad1.dpad_right && dpadRightToggle)
                dpadRightToggle = false;

            telemetry.addData("targetTagID", aprilTagAlignment.getTargetTagID());
            telemetry.addData("targetFound", aprilTagAlignment.getTargetFound());
            telemetry.addData("x error","%5.1f inches", aprilTagAlignment.getXError());
            telemetry.addData("y error","%5.1f inches", aprilTagAlignment.getYError());
            telemetry.addData("heading error","%3.0f degrees", aprilTagAlignment.getHeadingError());
            telemetry.addData("drivetrain power", Collections.max(Arrays.asList(drivetrain.getMotorPower())));
            telemetry.update();

            datalog.xError.set(aprilTagAlignment.getXError());
            datalog.yError.set(aprilTagAlignment.getYError());
            datalog.headingError.set(aprilTagAlignment.getHeadingError());
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

        drivetrain.move(forward, strafe, turn);
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

