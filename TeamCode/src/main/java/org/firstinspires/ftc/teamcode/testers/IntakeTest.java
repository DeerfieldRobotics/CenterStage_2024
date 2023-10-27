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
    private DcMotor intake;

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        while (opModeIsActive()) {
            double rightTriggerVal = gamepad1.right_trigger, leftTriggerVal = gamepad1.right_trigger;


            intake.setPower(rightTriggerVal);
        }
    }

    private void initialize() {
        intake = hardwareMap.get(DcMotor.class, "im");
//        i2 = hardwareMap.get(DcMotor.class, "i2");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void driveSet() {

    }

//    private void setPower(double rTP) {
//        if(rTP > 0){
//            i1.setPower(1);
//            i2.setPower(1);
//        }
//        else{
//            i1.setPower(0);
//            i2.setPower(0);
//        }
//    }

}
