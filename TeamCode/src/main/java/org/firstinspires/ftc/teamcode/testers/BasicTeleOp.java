package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * TELE OP CONTROLS
 * _for PS controller_
 * 
 * GAMEPAD 1:
 *   Left stick - Robot move/strafe
 *   Right stick - Robot rotation
 */

@TeleOp(name="BasicTeleOp", group="TeleOp")
public class BasicTeleOp extends LinearOpMode {

    private DcMotor fL, bL, fR, bR, intake, slideA, slideB;
    private Servo intakeS, outtakeS;

    private double oldServoPosition = 0.0;

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
        slideA = hardwareMap.get(DcMotor.class, "sa");
        slideB = hardwareMap.get(DcMotor.class, "sb");
        intake = hardwareMap.get(DcMotor.class, "im");

        intakeS = hardwareMap.get(Servo.class, "is");
        outtakeS = hardwareMap.get(Servo.class, "os");

        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);

        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void driveSet() {
        double rs_x = gamepad1.right_stick_x;
        double ls_y = -gamepad1.left_stick_y; 
        double ls_x = gamepad1.left_stick_x;

        double rt = gamepad1.right_trigger;

        boolean rb = gamepad1.right_bumper;
        boolean lb = gamepad1.left_bumper;

        double ls_y2 = gamepad2.left_stick_y;

        // My source: just trust me bro
        double k = Math.max(Math.abs(ls_y) + Math.abs(ls_x) + Math.abs(rs_x), 1);
        
        double fRP = (ls_y - ls_x - rs_x) / k;
        double fLP = (ls_y + ls_x + rs_x) / k;
        double bRP = (ls_y + ls_x - rs_x) / k;
        double bLP = (ls_y - ls_x + rs_x) / k;

        setPower(fRP, fLP, bRP, bLP, rt > 0.0, rb, lb);
    }

    private void setPower(double fRP, double fLP, double bRP, double bLP, boolean slide, boolean rb, boolean lb) {
        fR.setPower(fRP);
        fL.setPower(fLP);
        bR.setPower(bRP);
        bL.setPower(bLP);
        intake.setPower(slide ? 1 : 0);

        if (rb) {
            intakeS.setPosition(oldServoPosition+=0.01);
        }

        if (lb) {
            intakeS.setPosition(oldServoPosition-=0.01);
        }
    }
}