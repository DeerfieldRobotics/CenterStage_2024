package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="SlideTester", group="TeleOp")
public class SlideTester extends LinearOpMode {

    private DcMotor sa, sb;

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        while (opModeIsActive()) {
            double ly = gamepad2.left_stick_y;
            sa.setPower(ly);
            sb.setPower(ly);

            telemetry.addData("ticks", sa.getCurrentPosition());
            telemetry.update();
        }
    }

    private void initialize() {

        sa = hardwareMap.get(DcMotor.class, "sa");

        sa.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sa.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sa.setDirection(DcMotorSimple.Direction.REVERSE);
        sa.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sb = hardwareMap.get(DcMotor.class, "sb");

        sb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sb.setDirection(DcMotorSimple.Direction.REVERSE);
        sb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}