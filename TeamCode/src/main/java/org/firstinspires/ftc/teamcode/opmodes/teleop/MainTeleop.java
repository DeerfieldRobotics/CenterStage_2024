package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.DrivetrainKotlin;
import org.firstinspires.ftc.teamcode.utils.IntakeKotlin;
import org.firstinspires.ftc.teamcode.utils.Launcher;
import org.firstinspires.ftc.teamcode.utils.OuttakeKotlin;
import org.firstinspires.ftc.teamcode.utils.SlideKotlin;

@TeleOp(name = "Main Teleop", group = "a")
public class MainTeleop extends LinearOpMode {
    private DrivetrainKotlin drivetrain;
    private IntakeKotlin intake;
    private OuttakeKotlin outtake;
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

    SampleMecanumDrive drive;

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
    private boolean dpadLeftToggle = false;

    private boolean runToPos = false;
    private double lastTickTime = 0;
    private double avgTickTime = 0;
    private int tickCount = 0;
    private ElapsedTime runtime = new ElapsedTime();

    private IMU imu;
    // TODO fix parameters for c-hub placement
    private IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP
    ));

    private void driveNormal() {
        speedMult = .7+0.3 * gamepad1.right_trigger-0.5*gamepad1.left_trigger;

        gamepad1.rumble(gamepad1.left_trigger>0.5?(gamepad1.left_trigger-0.5)/.4:0.0,gamepad1.right_trigger>0.4?(gamepad1.right_trigger-0.4)/0.8:0.0,50);

        double forward = gamepad1.left_stick_y * forwardMult * speedMult;
        double turn = -gamepad1.right_stick_x * turnMult * speedMult;
        double strafe = -gamepad1.left_stick_x * strafeMult * speedMult;

        drivetrain.move(forward, strafe, turn);
    }
    //mack poop
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

    //kevin good
    private void driveRRCentered(){
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -0.8*gamepad1.left_stick_y,
                -0.8*gamepad1.left_stick_x
        ).rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        0.6*gamepad1.right_stick_x
                )
        );

        if(gamepad1.ps){
            drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(90)));
        }

        // Update everything. Odometry. Etc.
        drive.update();

        // Print pose to telemetry
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        runtime = new ElapsedTime();

        while (opModeIsActive()) {
            tickCount++;
            telemetry.addData("Tick Time", runtime.milliseconds()-lastTickTime);
            lastTickTime = runtime.milliseconds();
            avgTickTime = (avgTickTime*(tickCount-1)+runtime.milliseconds()-lastTickTime)/tickCount;
            telemetry.addData("Avg Tick Time", avgTickTime);

//            if (gamepad1.ps) {
//                imu.resetYaw();
//            }
//            driveCentered();
            driveNormal();
//            driveRRCentered();

            slide();
            intake();
            launcher();

            telemetry.addData("Drivetrain Average Current", drivetrain.getAvgCurrent());
            telemetry.addData("Slide Average Current", slide.getAvgCurrent());
            telemetry.addData("slide ticks", slide.getPosition()[0]);
            telemetry.addData("Intake Servo Pos: ", intake.getIntakePos());
            //telemetry.addData("Intake Motor Pos: ", intake.getIntakeMotor().getCurrentPosition());
            telemetry.addData("BOTTOMOUT: ", slide.getBottomOut());

            telemetry.update();
        }

    }
    // In MainTeleop.java
    private void slide() {
        slide.setPower(gamepad2.left_stick_y*l2Sensitivity/l2Max);
        slide.update();
    }
    private void intake() {
        //TODO: add vibration feedback when intake is fully opened and closed
        //Set intake values and clamp them between 0 and 1
        double rightTrigger = Math.max((gamepad2.right_trigger-rTriggerStart)/(rTriggerEnd-rTriggerStart),0);
        double leftTrigger = Math.max((gamepad2.left_trigger-lTriggerStart)/(lTriggerEnd-lTriggerStart),0);
        intake.intake(0.8*(rightTrigger-leftTrigger));

        //Outtake Code
        outtake.outtakeAngleAdjust(gamepad2.right_stick_y);

        if(gamepad2.dpad_up)
            intake.changeIntakeServo(.5);
        if(gamepad2.dpad_down)
            intake.changeIntakeServo(-.5);

//        if(gamepad2.dpad_left && !dpadLeftToggle) {
//            dpadLeftToggle = true;
//            slide.threadKill();
//            outtake.threadKill();
//            intake.threadKill();
//        }
//        if(!gamepad2.dpad_left) {
//            dpadLeftToggle = false;
//        }

        if(gamepad1.right_trigger>0.3 && intake.getServoPosition()==IntakeKotlin.IntakePositions.INTAKE)
            intake.setServoPosition(IntakeKotlin.IntakePositions.DRIVE);

        if (gamepad2.circle && !circleToggle) { // on circle press, outtake toggles
            gamepad2.rumbleBlips(1);
            circleToggle = true;
            outtake.setOuttakeProcedureTarget(OuttakeKotlin.OuttakePositions.OUTSIDE);
            intake.setServoPosition(IntakeKotlin.IntakePositions.HIGH);
        }
        if (!gamepad2.circle) {
            circleToggle = false;
        }

        //arm code
        if (gamepad2.triangle&&!triangleToggle) { // arm override
            triangleToggle = true;
            if (outtake.getOuttakePosition() == OuttakeKotlin.OuttakePositions.OUTSIDE) {
                outtake.setOuttakeProcedureTarget(OuttakeKotlin.OuttakePositions.INSIDE);
            } else {
                outtake.setOuttakeProcedureTarget(OuttakeKotlin.OuttakePositions.OUTSIDE);
            }
        }
        if (!gamepad2.triangle) {
            triangleToggle = false;
        }

        if(gamepad2.cross && !crossToggle) { //resets arm
            crossToggle = true;
            outtake.setOuttakeProcedureTarget(OuttakeKotlin.OuttakePositions.INSIDE);
            outtake.setGateClosed(true);
        }
        if(!gamepad2.cross) {
            crossToggle = false;
        }
        if(gamepad2.square && !squareToggle) { //gate override
            squareToggle = true;
            outtake.setGateClosed(!outtake.getGateClosed());
        }
        if(!gamepad2.square) {
            squareToggle = false;
        }

        if(gamepad2.right_bumper && !rightBumperToggle) {
            if(outtake.getOuttakePosition()!=OuttakeKotlin.OuttakePositions.OUTSIDE)
                outtake.setOuttakeProcedureTarget(OuttakeKotlin.OuttakePositions.TRANSFER);
            outtake.setGateClosed(false);
            gamepad2.rumble(1.0,1.0,100);
            rightBumperToggle = true;
            intake.transfer();
        }
        if (!gamepad2.right_bumper) {
            rightBumperToggle = false;
        }

        if(gamepad2.left_bumper && !leftBumperToggle) {
            if(intake.getServoPosition()!=IntakeKotlin.IntakePositions.INTAKE)
                intake.setServoPosition(IntakeKotlin.IntakePositions.INTAKE);
            else
                intake.setServoPosition(IntakeKotlin.IntakePositions.DRIVE);
        }
        if (!gamepad2.left_bumper) {
            leftBumperToggle = false;
        }
        intake.update();
        outtake.update();
    }

    public void launcher() {
        if (gamepad2.share)
            launcher.fire();
        else
            launcher.load();
    }


    public void initialize() {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        drive = new SampleMecanumDrive(hardwareMap);

        slide = new SlideKotlin(hardwareMap);
        outtake = new OuttakeKotlin(hardwareMap, slide);
        drivetrain = new DrivetrainKotlin(hardwareMap);
        intake = new IntakeKotlin(hardwareMap);
        launcher = new Launcher(hardwareMap);

        //slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(90)));

        drive.reverseMotors();

        waitForStart();
    }

}