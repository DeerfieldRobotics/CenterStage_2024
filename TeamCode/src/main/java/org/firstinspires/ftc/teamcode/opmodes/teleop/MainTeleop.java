package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.detection.AllianceHelper;
import org.firstinspires.ftc.teamcode.utils.detection.AprilTagAlignment;
import org.firstinspires.ftc.teamcode.utils.hardware.Intake;
import org.firstinspires.ftc.teamcode.utils.hardware.Launcher;
import org.firstinspires.ftc.teamcode.utils.hardware.Outtake;
import org.firstinspires.ftc.teamcode.utils.hardware.Slide;
import org.firstinspires.ftc.teamcode.roadrunner.drive.CogchampDrive;

@TeleOp(name = "Main Teleop", group = "a")
public class MainTeleop extends LinearOpMode {
    private CogchampDrive drivetrain;
    private Intake intake;
    private Outtake outtake;
    private Slide slide;
    private Launcher launcher;
    private AprilTagAlignment aprilTagAlignment;

    private boolean crossToggle = false;
    private boolean triangleToggle = false;
    private boolean squareToggle = false;
    private boolean circleToggle = false;
    private boolean rightBumperToggle = false;
    private double lastTickTime = 0;
    private double avgTickTime = 0;
    private int tickCount = 0;
    private double boardX = 0;
    public static AllianceHelper.Alliance alliance = null;
    private boolean robotAligned = false;

    private void driveNormal() {
        if(gamepad1.left_bumper) {
            boardX += 0.1*gamepad1.left_stick_x;
            if(boardX > 10.5) boardX = 10.5;
            if(boardX < -10.5) boardX = -10.5;
            aprilTagAlignment.setTargetX(boardX);
            aprilTagAlignment.update();
            aprilTagAlignment.alignRobotToBackboard(drivetrain);
            if(robotAligned != aprilTagAlignment.robotAligned()) {
                robotAligned = aprilTagAlignment.robotAligned();
                gamepad1.rumbleBlips(1);
            }
        }
        else {
            //driving values
            double speedMult = .7 + 0.3 * gamepad1.right_trigger - 0.3 * gamepad1.left_trigger;

            gamepad1.rumble(gamepad1.left_trigger > 0.5 ? (gamepad1.left_trigger - 0.5) / .4 : 0.0, gamepad1.right_trigger > 0.4 ? (gamepad1.right_trigger - 0.4) / 0.8 : 0.0, 50);

            double forwardMult = 1;
            double turnMult = .75;
            double strafeMult = 1;

            double forward = -gamepad1.left_stick_y * forwardMult * speedMult;
            double turn = -gamepad1.right_stick_x * turnMult * speedMult;
            double strafe = -gamepad1.left_stick_x * strafeMult * speedMult;

            drivetrain.setWeightedDrivePower(
                    new Pose2d(
                            forward,
                            strafe,
                            turn
                    )
            );
        }
    }

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        ElapsedTime runtime = new ElapsedTime();

        while (opModeIsActive()) {

            driveNormal();

            slide();
            intake();
            outtake();
            launcher();

            intake.update();
            outtake.update();
            slide.update();
            aprilTagAlignment.update();

            telemetry.addLine("--------15118 Teleop-------");
            telemetry.addData("CogchampDrive Average Current", drivetrain.getAverageCurrent());
            telemetry.addData("Slide Average Current", slide.getAvgCurrent());
            telemetry.addData("Slide Ticks", slide.getPosition()[0]);
            telemetry.addData("Intake Servo Pos: ", intake.getIntakePos());

            tickCount++;
            telemetry.addData("Tick Time", runtime.milliseconds()-lastTickTime);
            lastTickTime = runtime.milliseconds();
            avgTickTime = (avgTickTime*(tickCount-1)+ runtime.milliseconds()-lastTickTime)/tickCount;
            telemetry.addData("Avg Tick Time", avgTickTime);

            telemetry.update();
        }

    }
    private void slide() {
        //stick sensitivity values
        double l2Max = 0.8;
        slide.setPower(gamepad2.left_stick_y / l2Max);
    }
    private void intake() {
        double rTriggerStart = 0.05;
        double rTriggerEnd = 0.6;
        double rightTrigger = Math.max((gamepad2.right_trigger - rTriggerStart) / (rTriggerEnd - rTriggerStart), 0);

        double lTriggerStart = 0.05;
        double lTriggerEnd = 0.6;
        double leftTrigger = Math.max((gamepad2.left_trigger - lTriggerStart) / (lTriggerEnd - lTriggerStart), 0);

        intake.intake(0.8 * (rightTrigger - leftTrigger));

        if(gamepad2.dpad_up)
            intake.changeIntakeServo(-.5);
        if(gamepad2.dpad_down)
            intake.changeIntakeServo(.5);

        if(gamepad1.right_trigger>0.3 && intake.getServoPosition()== Intake.IntakePositions.INTAKE)
            intake.setServoPosition(Intake.IntakePositions.DRIVE);

        if(gamepad2.left_bumper)
            intake.setBoosterServoPower(1.0);
        else if(leftTrigger > 0.1)
            intake.setBoosterServoPower(-leftTrigger);
        else
            intake.setBoosterServoPower(0.0);


        if(gamepad2.right_bumper && !rightBumperToggle) {
            if(outtake.getOuttakePosition()!= Outtake.OuttakePositions.OUTSIDE)
                outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.TRANSFER);
            outtake.setGateClosed(false);
            gamepad2.rumble(1.0,1.0,100);
            rightBumperToggle = true;
            intake.transfer();
        }
        if (!gamepad2.right_bumper) { rightBumperToggle = false; }
    }
    private void outtake() {
        //Outtake Code
        outtake.outtakeAngleAdjust(gamepad2.right_stick_y);

        if (gamepad2.circle && !circleToggle) { // Outtake Procedure to Outside
            gamepad2.rumbleBlips(1);
            circleToggle = true;
            outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.OUTSIDE);
            intake.setServoPosition(Intake.IntakePositions.HIGH);
        }
        else if (!gamepad2.circle) { circleToggle = false; }

        if (gamepad2.triangle&&!triangleToggle) { // Outtake Toggle
            triangleToggle = true;
            if (outtake.getOuttakePosition() == Outtake.OuttakePositions.OUTSIDE) {
                outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.INSIDE);
            } else {
                outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.OUTSIDE);
            }
        }
        else if (!gamepad2.triangle) { triangleToggle = false; }

        if(gamepad2.cross && !crossToggle) { // Outtake Procedure to Inside
            crossToggle = true;
            outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.INSIDE);
            outtake.setGateClosed(true);
        }
        else if(!gamepad2.cross) { crossToggle = false; }

        if(gamepad2.square && !squareToggle) { // Gate Toggle
            squareToggle = true;
            outtake.setGateClosed(!outtake.getGateClosed());
        }
        else if(!gamepad2.square) { squareToggle = false; }
    }

    public void launcher() {
        if (gamepad2.share)
            launcher.fire();
        else
            launcher.load();
    }

    public void initialize() {
        while(!isStopRequested() && alliance == null) {
            if(gamepad1.dpad_up || gamepad2.dpad_up) {
                alliance = AllianceHelper.Alliance.RED;
                AllianceHelper.alliance = AllianceHelper.Alliance.RED;
                break;
            }
            else if(gamepad1.dpad_down || gamepad2.dpad_down) {
                alliance = AllianceHelper.Alliance.BLUE;
                AllianceHelper.alliance = AllianceHelper.Alliance.BLUE;
                break;
            }
            telemetry.addData("Alliance", "Select Alliance");
            telemetry.addData("↑", "Red");
            telemetry.addData("↓", "Blue");
            telemetry.update();
        }
        telemetry.clear();
        slide = new Slide(hardwareMap);
        outtake = new Outtake(hardwareMap, slide);
        drivetrain = new CogchampDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        launcher = new Launcher(hardwareMap);
        aprilTagAlignment = new AprilTagAlignment(hardwareMap.get(WebcamName.class, "Webcam 1"), 0.0, 12.0, 0.0, AllianceHelper.alliance,
                (new PIDController(0.0174, 0.0, 0.0)), //x PID controller
                (new PIDController(0.0174, 0.0, 0.0)), //y PID controller
                (new PIDController(0.0174, 0.0, 0.0))); //heading PID controller

        drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


//        if (AprilTagAlignment.Alliance.RED == alliance) {
//            gamepad1.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
//            gamepad2.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
//        } else {
//            gamepad1.setLedColor(0, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
//            gamepad2.setLedColor(0, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
//        }

        waitForStart();
    }
}