package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.utils.hardware.Intake;
import org.firstinspires.ftc.teamcode.utils.hardware.Launcher;
import org.firstinspires.ftc.teamcode.utils.hardware.Outtake;
import org.firstinspires.ftc.teamcode.utils.hardware.Slide;
import org.firstinspires.ftc.teamcode.roadrunner.drive.CogchampDrive;

import java.util.List;

@TeleOp(name = "Main Teleop", group = "a")
public class MainTeleop extends LinearOpMode {
    private CogchampDrive drivetrain;
    private Intake intake;
    private Outtake outtake;
    private Slide slide;
    private Launcher launcher;

    private boolean crossToggle = false;
    private boolean triangleToggle = false;
    private boolean squareToggle = false;
    private boolean circleToggle = false;
    private boolean rightBumperToggle = false;
    private boolean dpadDownToggle = false;
    private boolean dpadUpToggle = false;

    private boolean hookReleased = false;
    private long millisAfterRelease = 0;

    private void driveNormal() {
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
        }

    }
    private void slide() {
        //stick sensitivity values
        double l2Max = 0.8;
        slide.setPower(gamepad2.left_stick_y / l2Max);

        if(gamepad2.left_stick_button && gamepad2.right_stick_button && !hookReleased) {
            slide.releaseHook();
            hookReleased = true;
            millisAfterRelease = System.currentTimeMillis();
        }
        if(hookReleased && System.currentTimeMillis() - millisAfterRelease > 500)
            slide.hookMotorPower(-gamepad2.right_stick_y);
    }
    private void intake() {
        double rTriggerStart = 0.05;
        double rTriggerEnd = 0.6;
        double rightTrigger = Math.max((gamepad2.right_trigger - rTriggerStart) / (rTriggerEnd - rTriggerStart), 0);

        double lTriggerStart = 0.05;
        double lTriggerEnd = 0.6;
        double leftTrigger = Math.max((gamepad2.left_trigger - lTriggerStart) / (lTriggerEnd - lTriggerStart), 0);

        intake.intake(0.8 * (rightTrigger - leftTrigger), gamepad2.left_bumper);

        if(gamepad2.dpad_up && !dpadUpToggle) {
            intake.intakePositionStepUp();
            gamepad2.rumble(0.4, 0.8, 100);
            dpadUpToggle = true;
        }
        if(!gamepad2.dpad_up) {
            dpadUpToggle = false;
        }
        if(gamepad2.dpad_down && !dpadDownToggle) {
            intake.intakePositionStepDown();
            gamepad2.rumble(0.8, 0.4, 100);
            dpadDownToggle = true;
        }
        if(!gamepad2.dpad_down) {
            dpadDownToggle = false;
        }


        if(gamepad1.right_trigger>0.3 && intake.getServoPosition()== Intake.IntakePositions.INTAKE)
            intake.setServoPosition(Intake.IntakePositions.DRIVE);

//        if(gamepad2.left_bumper)
//            intake.setBoosterServoPower(-1.0);
        if(leftTrigger > 0.1)
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
        if(!hookReleased)
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
        //BULK READS
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        //HARDWARE
        slide = new Slide(hardwareMap);
        outtake = new Outtake(hardwareMap, slide);
        drivetrain = new CogchampDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        launcher = new Launcher(hardwareMap);

        drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
    }
}