package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.utils.SlideKotlin;



@TeleOp(name="SlideTester", group="b")
public class SlideTester extends LinearOpMode {

    private SlideKotlin slide;

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        while (opModeIsActive()) {
            slide.setPower(gamepad2.left_stick_y);

            telemetry.addData("ticks", slide.getPosition()[0]);
            telemetry.update();
        }
    }

    private void initialize() {
        slide = new SlideKotlin(hardwareMap);
    }
}