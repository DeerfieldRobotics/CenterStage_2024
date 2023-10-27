package org.firstinspires.ftc.teamcode.testers;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "servotester")
public class ServoTest extends LinearOpMode {
    private Servo s;

    private double increment = 0.01;

    private double pos = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean[/*L, R*/] wasPressed = {false, false};

        initialize();

        waitForStart();

        while (opModeIsActive()) {
            double low = .62;
            double high = .7;

            s.setPosition(high - (high-low)*gamepad1.right_trigger);

            telemetry.addData("Pos", s.getPosition());
            telemetry.update();

//            boolean u = gamepad1.dpad_up;
//            boolean d = gamepad1.dpad_down;
//
//            if (u && wasPressed[0] == false) {
//                pos += increment;
//                updateServo(pos);
//                wasPressed[0] = true;
//            } else if (!u && wasPressed[0] == true) {
//                wasPressed[0] = false;
//            }
//
//            if (d && wasPressed[1] == false) {
//                pos -= increment;
//                updateServo(pos);
//                wasPressed[1] = true;
//            } else if (!d && wasPressed[1] == true) {
//                wasPressed[1] = false;
//            }
        }
    }

    public void initialize() {
        s = hardwareMap.get(Servo.class, "as");
    }

    public void updateServo(double n) {
        telemetry.addData("Pos", s.getPosition());
        telemetry.addData("Told", n);
        telemetry.update();
    }
}