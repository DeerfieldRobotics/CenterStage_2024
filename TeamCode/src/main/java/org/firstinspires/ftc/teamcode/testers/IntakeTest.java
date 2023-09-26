package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "NewIntakeTest", group = "TeleOp")
public class IntakeTest extends LinearOpMode {
    private DcMotor i1, i2;

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        while (opModeIsActive()) {
            driveSet();
        }
    }

    private void initialize() {
        i1 = hardwareMap.get(DcMotor.class, "i1");
        i2 = hardwareMap.get(DcMotor.class, "i2");

        i2.setDirection(DcMotorSimple.Direction.REVERSE);
        i1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        i2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void driveSet() {
        double rightTriggerVal = gamepad1.right_trigger;

        setPower(rightTriggerVal);
    }

    private void setPower(double rTP) {
        if(rTP > 0){
            i1.setPower(1);
            i2.setPower(1);
        }
        else{
            i1.setPower(0);
            i2.setPower(0);
        }
    }

}
