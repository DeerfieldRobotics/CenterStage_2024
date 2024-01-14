package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.utils.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.hardware.Intake;
import org.firstinspires.ftc.teamcode.utils.hardware.Launcher;
import org.firstinspires.ftc.teamcode.utils.hardware.Outtake;
import org.firstinspires.ftc.teamcode.utils.hardware.Slide;

@TeleOp(name = "Main Teleop", group = "a")
public class MainTeleop extends LinearOpMode {
    private Drivetrain drivetrain;
    private Intake intake;
    private Outtake outtake;
    private Slide slide;
    private Launcher launcher;

    private boolean crossToggle = false;
    private boolean triangleToggle = false;
    private boolean squareToggle = false;
    private boolean circleToggle = false;
    private boolean rightBumperToggle = false;
    private boolean leftBumperToggle = false;
    private boolean dpadLeftToggle = false;
    private double lastTickTime = 0;
    private double avgTickTime = 0;
    private int tickCount = 0;

    private void driveNormal() {
        //driving values
        double speedMult = .7 + 0.3 * gamepad1.right_trigger - 0.3 * gamepad1.left_trigger;

        gamepad1.rumble(gamepad1.left_trigger>0.5?(gamepad1.left_trigger-0.5)/.4:0.0,gamepad1.right_trigger>0.4?(gamepad1.right_trigger-0.4)/0.8:0.0,50);

        double forwardMult = 1;
        double turnMult = .75;
        double strafeMult = 1;

        double forward = gamepad1.left_stick_y * forwardMult * speedMult;
        double turn = -gamepad1.right_stick_x * turnMult * speedMult;
        double strafe = -gamepad1.left_stick_x * strafeMult * speedMult;

        drivetrain.move(forward, strafe, turn);
    }

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();
        ElapsedTime runtime = new ElapsedTime();

        while (opModeIsActive()) {
            tickCount++;
            telemetry.addData("Tick Time", runtime.milliseconds()-lastTickTime);
            lastTickTime = runtime.milliseconds();
            avgTickTime = (avgTickTime*(tickCount-1)+ runtime.milliseconds()-lastTickTime)/tickCount;
            telemetry.addData("Avg Tick Time", avgTickTime);

            driveNormal();

            slide();
            intake();
            launcher();

            intake.update();
            outtake.update();
            slide.update();

            telemetry.addData("Drivetrain Average Current", drivetrain.getAvgCurrent());
            telemetry.addData("Slide Average Current", slide.getAvgCurrent());
            telemetry.addData("slide ticks", slide.getPosition()[0]);
            telemetry.addData("Intake Servo Pos: ", intake.getIntakePos());
            //telemetry.addData("Intake Motor Pos: ", intake.getIntakeMotor().getCurrentPosition());
            telemetry.addData("BOTTOMOUT: ", slide.getBottomOut());

            telemetry.update();
        }

    }
    private void slide() {
        //stick sensitivity values
        double l2Sensitivity = 1;
        double l2Max = 0.8;
        slide.setPower(gamepad2.left_stick_y* l2Sensitivity / l2Max);
    }
    private void intake() {
        //TODO: add vibration feedback when intake is fully opened and closed
        //Set intake values and clamp them between 0 and 1
        //Sensitivity values for triggers of gamepad 2
        double rTriggerStart = 0.05;
        double rTriggerEnd = 0.6;

        double rightTrigger = Math.max((gamepad2.right_trigger- rTriggerStart)/(rTriggerEnd - rTriggerStart),0);

        double lTriggerStart = 0.05;
        double lTriggerEnd = 0.6;

        double leftTrigger = Math.max((gamepad2.left_trigger- lTriggerStart)/(lTriggerEnd - lTriggerStart),0);

        intake.intake(0.8*(rightTrigger-leftTrigger));

        //Outtake Code
        outtake.outtakeAngleAdjust(gamepad2.right_stick_y);

        if(gamepad2.dpad_up)
            intake.changeIntakeServo(.5);
        if(gamepad2.dpad_down)
            intake.changeIntakeServo(-.5);

        if(gamepad1.right_trigger>0.3 && intake.getServoPosition()== Intake.IntakePositions.INTAKE)
            intake.setServoPosition(Intake.IntakePositions.DRIVE);

        if (gamepad2.circle && !circleToggle) { // on circle press, outtake toggles
            gamepad2.rumbleBlips(1);
            circleToggle = true;
            outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.OUTSIDE);
            intake.setServoPosition(Intake.IntakePositions.HIGH);
        }
        if (!gamepad2.circle) {
            circleToggle = false;
        }

        //arm code
        if (gamepad2.triangle&&!triangleToggle) { // arm override
            triangleToggle = true;
            if (outtake.getOuttakePosition() == Outtake.OuttakePositions.OUTSIDE) {
                outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.INSIDE);
            } else {
                outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.OUTSIDE);
            }
        }
        if (!gamepad2.triangle) {
            triangleToggle = false;
        }

        if(gamepad2.cross && !crossToggle) { //resets arm
            crossToggle = true;
            outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.INSIDE);
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
            if(outtake.getOuttakePosition()!= Outtake.OuttakePositions.OUTSIDE)
                outtake.setOuttakeProcedureTarget(Outtake.OuttakePositions.TRANSFER);
            outtake.setGateClosed(false);
            gamepad2.rumble(1.0,1.0,100);
            rightBumperToggle = true;
            intake.transfer();
        }
        if (!gamepad2.right_bumper) {
            rightBumperToggle = false;
        }

        if(gamepad2.left_bumper && !leftBumperToggle) {
            if(intake.getServoPosition()!= Intake.IntakePositions.INTAKE)
                intake.setServoPosition(Intake.IntakePositions.INTAKE);
            else
                intake.setServoPosition(Intake.IntakePositions.DRIVE);
        }
        if (!gamepad2.left_bumper) {
            leftBumperToggle = false;
        }
    }

    public void launcher() {
        if (gamepad2.share)
            launcher.fire();
        else
            launcher.load();
    }

    public void initialize() {
        slide = new Slide(hardwareMap);
        outtake = new Outtake(hardwareMap, slide);
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        launcher = new Launcher(hardwareMap);

        drivetrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
    }
}