package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.DrivetrainKotlin;
import org.firstinspires.ftc.teamcode.utils.IntakeKotlin;
import org.firstinspires.ftc.teamcode.utils.SlideKotlin;

@TeleOp(name = "main teleop")
public class main_teleop extends LinearOpMode {
    private DcMotor sa, sb;
    private ElapsedTime runtime;
    private DrivetrainKotlin drivetrain;
    private IntakeKotlin intake;
    private SlideKotlin slide;
    //Sensitivity values for triggers of gamepad 2
    private final double rTriggerStart = 0.05;
    private final double rTriggerEnd = 0.6;
    private final double lTriggerStart = 0.05;
    private final double lTriggerEnd = 0.6;

    //stick sensitivity values
    private final double l1Sensitivity = 1;
    private final double r1Sensitivity = 1;
    private final double l2Sensitivity = 1;
    private final double r2Sensitivity = 1;
    //driving values
    private double speedMult;
    private double forwardMult = .7;
    private double turnMult = .65;
    private double strafeMult = .9;
    //servo position values
    private final double[] intakeServoPositions = {0.0, 0.2, 0.4, 0.6, 0.8, 1.0};
    private int servoCounter = 2;
    double currentOuttakePos = 1.0;
    double currentArmPos = 1.0;

    private boolean crossToggle = false;
    private boolean triangleToggle = false;
    private boolean squareToggle = false;
    private boolean circleToggle = false;
    private boolean rightBumperToggle = false;
    private boolean leftBumperToggle = false;

    private IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
    ));

    private void driveNormal () {
        speedMult = .7+0.3 * gamepad1.right_trigger-0.5*gamepad1.left_trigger;

        double forward = gamepad1.left_stick_y * forwardMult * speedMult;
        double turn = -gamepad1.right_stick_x * turnMult * speedMult;
        double strafe = -gamepad1.left_stick_x * strafeMult * speedMult;

        drivetrain.move(forward, strafe, turn);
    }


    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        while (opModeIsActive()) {
//            if (gamepad1.ps) {
//                imu.resetYaw();
//            }

            //driveSetCentered();
            driveNormal();
            telemetry.addData("Drivetrain Moving", (drivetrain.isBusy() ? "true" : "false"));

            slide();
            intake();
            telemetry.addData("Drivetrain Current", drivetrain.getCurrent());
            telemetry.addData("Slide Current", slide.getCurrent());
            telemetry.addData("intakeServo", intake.getIntakePos());
            telemetry.addData("outtakeServo", intake.getOuttakePos());
            telemetry.addData("armServo", intake.getArmPos());
            telemetry.addData("crossToggle", crossToggle);

            telemetry.update();
        }

    }

    private void slide() {
        double slidePower = gamepad2.left_stick_y*l2Sensitivity;
        if (slidePower >= -0.8 && slidePower <= 0.8)
            slide.setPower(slidePower);
        else if (slidePower > 0.8)
            slide.setPower(1);
        else if (slidePower < -0.8)
            slide.setPower(-1);
        else
            slide.setPower(0);

        // telemetry.addData("Slide Current", slide.getCurrent());
    }

    private void intake() {
        //TODO: add vibration feedback when intake is fully opened and closed
        //Set intake values and clamp them between 0 and 1
        double rightTrigger = Math.max((gamepad2.right_trigger-rTriggerStart)/(rTriggerEnd-rTriggerStart),0);
        double leftTrigger = Math.max((gamepad2.left_trigger-lTriggerStart)/(lTriggerEnd-lTriggerStart),0);
        intake.intake(leftTrigger-rightTrigger);

        if (servoCounter != 5&&gamepad2.right_bumper) { //Changes intake servo values on release
            rightBumperToggle = true;
        }
        if (!gamepad2.right_bumper & rightBumperToggle)
        {
            rightBumperToggle = false;
            servoCounter++;
        }
        if (servoCounter != 0&&gamepad2.left_bumper)
        {
            leftBumperToggle = true;
        }
        if (!gamepad2.left_bumper & leftBumperToggle)
        {
            leftBumperToggle = false;
            servoCounter--;
        }
        intake.intakeServo(servoCounter);

        //Outtake Code
        if (gamepad2.cross) {
            crossToggle = true;
        }
        if (!gamepad2.cross & crossToggle)
        {
            crossToggle = false;
            intake.outtakeToggle();
        }

        //arm code
        if (gamepad2.triangle) {
            triangleToggle = true;
        }
        if (!gamepad2.triangle & triangleToggle)
        {
            triangleToggle = false;
            intake.armToggle();
        }
    }

    public void initialize() {
//        imu = hardwareMap.get(IMU.class, "imu");
//        imu.initialize(parameters);

        drivetrain = new DrivetrainKotlin(hardwareMap);
        intake = new IntakeKotlin(hardwareMap);
        slide = new SlideKotlin(hardwareMap);

        drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}