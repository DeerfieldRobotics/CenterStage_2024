package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

            driveSetCentered();

            if(gamepad2.right_trigger>0.2)
                intake.grip(true);
            else
                intake.grip(false);
            if(gamepad2.left_trigger>0.2)
                intake.turn(true);
            else
                intake.turn(false);
            slide.setPower((double)gamepad2.left_stick_y);
            telemetry.addData("Slide Current", slide.getCurrent());
        }
    }

    private void initialize() {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        drivetrain = new DrivetrainKotlin(hardwareMap);
        intake = new IntakeKotlin(hardwareMap);
    }

    private void driveSetCentered() {
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
}