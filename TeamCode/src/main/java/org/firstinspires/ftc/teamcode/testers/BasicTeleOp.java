package org.firstinspires.ftc.teamcode.testers;

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

    private DcMotor fL, bL, fR, bR, s, i1, i2;

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

        s = hardwareMap.get(DcMotor.class, "s");
        i1 = hardwareMap.get(DcMotor.class, "i1");
        i2 = hardwareMap.get(DcMotor.class, "i2");

        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);

        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        s.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        s.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        i2.setDirection(DcMotorSimple.Direction.REVERSE);
        i1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        i2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void driveSet() {
        double rs_x = gamepad1.right_stick_x;
        double ls_y = -gamepad1.left_stick_y; 
        double ls_x = gamepad1.left_stick_x;

        double rightTriggerVal = gamepad1.right_trigger;

        double ls_y2 = gamepad2.left_stick_y;

        double k = Math.max(Math.abs(ls_y) + Math.abs(ls_x) + Math.abs(rs_x), 1);
        
        double fRP = (ls_y - ls_x - rs_x) / k;
        double fLP = (ls_y + ls_x + rs_x) / k;
        double bRP = (ls_y + ls_x - rs_x) / k;
        double bLP = (ls_y - ls_x + rs_x) / k;

        setPower(fRP, fLP, bRP, bLP, rightTriggerVal);
    }

    private void setPower(double fRP, double fLP, double bRP, double bLP, double rTP) {
        fR.setPower(fRP);
        fL.setPower(fLP);
        bR.setPower(bRP);
        bL.setPower(bLP);
        if(rTP > 0){
            i1.setPower(1);
            i2.setPower(1);
        }
    }
}