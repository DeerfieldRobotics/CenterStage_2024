package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.DrivetrainKotlin;

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

    private DcMotor fL, bL, fR, bR;
    private DrivetrainKotlin drivetrain;

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
        }
    }

    private void initialize() {
        drivetrain = new DrivetrainKotlin(hardwareMap);
        fR = hardwareMap.get(DcMotor.class, "fr");
        fL = hardwareMap.get(DcMotor.class, "fl");
        bR = hardwareMap.get(DcMotor.class, "br");
        bL = hardwareMap.get(DcMotor.class, "bl");

        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(parameters);
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

        setPower(fRP, fLP, bRP, bLP);
    }

    private void setPower(double fRP, double fLP, double bRP, double bLP) {
        fR.setPower(fRP);
        fL.setPower(fLP);
        bR.setPower(bRP);
        bL.setPower(bLP);
    }
}