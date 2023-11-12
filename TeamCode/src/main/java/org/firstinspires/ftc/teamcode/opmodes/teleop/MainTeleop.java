package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.DrivetrainKotlin;
import org.firstinspires.ftc.teamcode.utils.IntakeKotlin;
import org.firstinspires.ftc.teamcode.utils.Launcher;
import org.firstinspires.ftc.teamcode.utils.SlideKotlin;

@TeleOp(name = "Main Teleop", group = "a")
public class MainTeleop extends LinearOpMode {
    private DrivetrainKotlin drivetrain;
    private IntakeKotlin intake;
    private SlideKotlin slide;
    private Launcher launcher;
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
    private double l2Max = 0.8;
    //driving values
    private double speedMult;
    private double forwardMult = .7;
    private double turnMult = .65;
    private double strafeMult = .9;
    //intake time values
    long timeSinceArm = 0;
    long timeSinceOuttake = 0;

    //servo position values
//    private final double[] intakeServoPositions = {0.0, 0.2, 0.4, 0.6, 0.8, 1.0};
    private int servoCounter = 2;
    double currentOuttakePos = 1.0;
    double currentArmPos = 1.0;

    private boolean crossToggle = false;
    private boolean triangleToggle = false;
    private boolean squareToggle = false;
    private boolean circleToggle = false;
    private boolean rightBumperToggle = false;
    private boolean leftBumperToggle = false;

    private boolean runToPos = false;
    private ElapsedTime runtime = new ElapsedTime();

    private IMU imu;
    // TODO fix parameters for c-hub placement
    private IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP
    ));

    private void driveNormal() {
        speedMult = .7+0.3 * gamepad1.right_trigger-0.5*gamepad1.left_trigger;

        double forward = gamepad1.left_stick_y * forwardMult * speedMult;
        double turn = -gamepad1.right_stick_x * turnMult * speedMult;
        double strafe = -gamepad1.left_stick_x * strafeMult * speedMult;

        drivetrain.move(forward, strafe, turn);
    }

    private void driveCentered() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.options) { imu.resetYaw(); }

        double h = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        telemetry.addData("IMU Heading", Math.toDegrees(h));

        x = x * Math.cos(-h) - y * Math.sin(-h);
        y = x * Math.sin(-h) + y * Math.cos(-h);

        double flp = y + x + rx;
        double blp = y - x + rx;
        double frp = y - x - rx;
        double brp = y + x - rx;

        drivetrain.setMotorPower(flp, frp, blp, brp);
    }

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        while (opModeIsActive()) {
//            if (gamepad1.ps) {
//                imu.resetYaw();
//            }

//            driveCentered();
            driveNormal();
            telemetry.addData("Drivetrain Moving", (drivetrain.isBusy() ? "true" : "false"));

            slide();
            intake();
            launcher();

            telemetry.addData("Drivetrain Average Current", drivetrain.getAvgCurrent());
            telemetry.addData("Slide Average Current", slide.getAvgCurrent());
            telemetry.addData("slide ticks", slide.getPosition()[0]);
            telemetry.addData("slide isBusy", slide.isBusy());
            telemetry.addData("Intake Servo Pos: ", intake.getIntakePos());

            telemetry.update();
        }

    }

    private void slide() {
        double slidePower = gamepad2.left_stick_y*l2Sensitivity/l2Max;
        if(slidePower!=0) {
            slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (slidePower >= 1)
                slide.setPower(1);
            else if (slidePower <= -1)
                slide.setPower(-1);
            else
                slide.setPower(slidePower);
        }
        else
            slide.setPower(0);

        telemetry.addData("Slide Current", slide.getCurrent());
    }

    private void intake() {
        //TODO: add vibration feedback when intake is fully opened and closed
        //Set intake values and clamp them between 0 and 1
        double rightTrigger = Math.max((gamepad2.right_trigger-rTriggerStart)/(rTriggerEnd-rTriggerStart),0);
        double leftTrigger = Math.max((gamepad2.left_trigger-lTriggerStart)/(lTriggerEnd-lTriggerStart),0);
        telemetry.addData("intakePower", rightTrigger);
        intake.intake(0.4*(rightTrigger-leftTrigger));

//        if (servoCounter != 3&&gamepad2.right_bumper && !rightBumperToggle) { //Changes intake servo values on release
//            rightBumperToggle = true;
//            servoCounter++;
//            intake.intakeServo(servoCounter);
//        }
//        if (!gamepad2.right_bumper) {
//            rightBumperToggle = false;
//        }
//        if (servoCounter != 0&&gamepad2.left_bumper && !leftBumperToggle) {
//            leftBumperToggle = true;
//            servoCounter--;
//            intake.intakeServo(servoCounter);
//        }
//        if (!gamepad2.left_bumper) {
//            leftBumperToggle = false;
//        }

        intake.changeIntakeServo(gamepad2.right_stick_y);


        //Outtake Code


        if (gamepad2.circle && !circleToggle) { // on circle press, outtake toggles
            circleToggle = true;
            intake.outtakeToggle();
        }
        if (!gamepad2.circle) {
            circleToggle = false;
        }

        //arm code
        if (gamepad2.triangle&&!triangleToggle) { // on triangle press, arm toggles
            triangleToggle = true;
            intake.armToggle();
        }
        if (!gamepad2.triangle) {
            triangleToggle = false;
        }

        if(gamepad2.cross && !crossToggle) { //brings intake shit out
            crossToggle = true;
            intake.intakeProcedure(true, slide.getMinSlideHeight());
        }
        if(!gamepad2.cross) {
            crossToggle = false;
        }
        if(gamepad2.square && !squareToggle) { //brings intake shit in
            squareToggle = true;
            intake.intakeProcedure(false, slide.getMinSlideHeight()); //works w
        }
        if(!gamepad2.square) {
            squareToggle = false;
        }
    }

    public void launcher() {
        if (gamepad2.share)
            launcher.fire();
//        else
//            launcher.load();
    }


    public void initialize() {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        drivetrain = new DrivetrainKotlin(hardwareMap);
        slide = new SlideKotlin(hardwareMap);
        intake = new IntakeKotlin(hardwareMap, slide);
        launcher = new Launcher(hardwareMap);

        //slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.getOuttakeServo().setPosition(0.34);
        intake.getIntakeServo().setPosition(0.1);
    }

}