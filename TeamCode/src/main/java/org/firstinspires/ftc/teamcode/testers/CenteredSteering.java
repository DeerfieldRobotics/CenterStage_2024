package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.DrivetrainKotlin;
import org.firstinspires.ftc.teamcode.utils.IntakeKotlin;
import org.firstinspires.ftc.teamcode.utils.SlideKotlin;

/**
 * TELE OP CONTROLS
 * _for PS controller_
 * 
 * GAMEPAD 1:
 *   Left stick - Robot move/strafe
 *   Right stick - Robot rotation
 *   PS - Set robot orientation as forward
 */

@TeleOp(name="Centric Steering", group="TeleOp")
public class CenteredSteering extends LinearOpMode {
    private IMU imu;
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


    private IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.UP,
        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
    ));

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.ps) {
                imu.resetYaw();
            }

            //driveSetCentered();
            driveNormal();
            intake();
            slide();

            telemetry.update();
        }
    }

    private void initialize() {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        drivetrain = new DrivetrainKotlin(hardwareMap);
        intake = new IntakeKotlin(hardwareMap);
        slide = new SlideKotlin(hardwareMap);
    }

    private void driveSetCentered() {
        //TODO maybe make sensitivities with sticks for movement
        //TODO add trigger speed control

        double rs_x = gamepad1.right_stick_x;
        double ls_y = -gamepad1.left_stick_y;
        double ls_x = gamepad1.left_stick_x;

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


        // https://matthew-brett.github.io/teaching/rotation_2d.html
        double rotX = ls_x * Math.cos(-heading) - ls_y * Math.sin(-heading);
        double rotY = ls_x * Math.sin(-heading) + ls_y * Math.cos(-heading);

        double k = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rs_x), 1);

        double fRP = (rotY - rotX - rs_x) / k;
        double fLP = (rotY + rotX + rs_x) / k;
        double bRP = (rotY + rotX - rs_x) / k;
        double bLP = (rotY - rotX + rs_x) / k;

        drivetrain.setMotorPower(fRP, fLP, bRP, bLP);
    }
    private void driveNormal () {
        double rs_x = gamepad1.right_stick_x;
        double ls_y = -gamepad1.left_stick_y;
        double ls_x = gamepad1.left_stick_x;

        double ls_y2 = gamepad2.left_stick_y;

        double k = Math.max(Math.abs(ls_y) + Math.abs(ls_x) + Math.abs(rs_x), 1);

        double fRP = (ls_y - ls_x - rs_x) / k;
        double fLP = (ls_y + ls_x + rs_x) / k;
        double bRP = (ls_y + ls_x - rs_x) / k;
        double bLP = (ls_y - ls_x + rs_x) / k;

        drivetrain.setMotorPower(fRP, fLP, bRP, bLP);
    }

    private void intake() {
        //TODO: add vibration feedback when intake is fully opened and closed
        //Set intake values and clamp them between 0 and 1
        double g = (gamepad2.right_trigger-rTriggerStart)/(rTriggerEnd-rTriggerStart);
        if(g>0&&g<=1)
            intake.grip(g);
        else if(g>1)
            intake.grip(1);
        else
            intake.grip(0);

        double t = (gamepad2.left_trigger-lTriggerStart)/(lTriggerEnd-lTriggerStart);
        if(t>0&&t<=1)
            intake.turn(t);
        else if(t>1)
            intake.turn(1);
        else
            intake.turn(0);
    }

    private void slide() {
        double s = gamepad2.left_stick_y*l2Sensitivity;
        if(s>0&&s<=1)
            slide.setPower(s);
        else if(s>1)
            slide.setPower(1);
        else if(s<0&&s>=-1)
            slide.setPower(s);
        else if(s<-1)
            slide.setPower(-1);
        else
            slide.setPower(0);

        //telemetry.addData("Slide Current", slide.getCurrent());
    }
}