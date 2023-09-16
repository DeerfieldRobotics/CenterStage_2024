package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="SlideTester", group="TeleOp")
public class SlideTester extends LinearOpMode {

    private DcMotor s;

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        while (opModeIsActive()) {
            double ly = gamepad2.left_stick_y;
            s.setPower(ly);
            telemetry.addData("ticks", s.getCurrentPosition());
            telemetry.update();
        }
    }

    private void initialize() {

        s = hardwareMap.get(DcMotor.class, "s");

        s.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        s.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        s.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}