package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.utils.hardware.Slide;



@TeleOp(name="SlideTester", group="c")
public class SlideTester extends LinearOpMode {

    private Slide slide;

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        while (opModeIsActive()) {
            slide.setPower(gamepad2.left_stick_y);

            telemetry.addData("ticks", slide.getPosition()[0]);
            telemetry.addData("ticks 2", slide.getPosition()[1]);
            telemetry.update();
        }
    }

    private void initialize() {
        slide = new Slide(hardwareMap);
    }
}