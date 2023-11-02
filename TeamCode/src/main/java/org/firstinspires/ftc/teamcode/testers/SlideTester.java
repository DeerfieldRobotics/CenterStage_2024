package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.SlideKotlin;


@TeleOp(name="SlideTester", group="TeleOp")
public class SlideTester extends LinearOpMode {

    private SlideKotlin slide;

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        while (opModeIsActive()) {
            double ly = gamepad2.left_stick_y;
            slide.setPower(ly);

            telemetry.addData("ticks a", slide.getPosition()[0]);
            telemetry.addData("ticks b", slide.getPosition()[1]);
            telemetry.update();
        }
    }

    private void initialize() {
        slide = new SlideKotlin(hardwareMap);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}