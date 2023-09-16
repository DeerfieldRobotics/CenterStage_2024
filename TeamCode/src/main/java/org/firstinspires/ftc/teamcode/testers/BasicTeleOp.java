package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * TELE OP CONTROLS
 * _for PS controller_
 * 
 * GAMEPAD 1:
 *   Left stick - Robot move/strafe
 *   Right stick - Robot rotation
 */

@TeleOp(name="Base", group="TeleOp")
public class BasicTeleOp extends LinearOpMode {

    private DcMotor fL, bL, fR, bR;

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        while (opModeIsActive()) {
            driveSet();
        }
    }

    private void initialize() {
        fR = hardwareMap.get(DcMotor.class, "fr");
        fL = hardwareMap.get(DcMotor.class, "fl");
        bR = hardwareMap.get(DcMotor.class, "br");
        bL = hardwareMap.get(DcMotor.class, "bl");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void driveSetCentered() {
        double rs_x = gamepad1.right_stick_x;
        double ls_y = -gamepad1.left_stick_y; 
        double ls_x = gamepad1.left_stick_x;

        double k = Math.max(Math.abs(ls_y) + Math.abs(ls_x) + Math.abs(rs_x), 1);
        
        double fRP = (ls_y - ls_x - rs_x) / k;
        double fLP = (ls_y + ls_x + rs_x) / k;
        double bRP = (ls_y + ls_x - rs_x) / k;
        double bLP = (ls_y - ls_x + rs_x) / k;

        setPower(fRP, fLP, bRP, bLP);
    }

    private void setPower(double fRP, double fLP, double bRP, double bLP) {
        fR.setPower(fRP);
        fL.setPower(fLP);
        bR.setPower(bRP);
        bL.setPower(bLP);
    }
}