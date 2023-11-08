package org.firstinspires.ftc.teamcode.testers;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "servotester", group = "b")
public class ServoTest extends LinearOpMode {
    private Servo s;

    private double increment = 0.01;
    private boolean dpad_up = false;
    private boolean dpad_down = false;

    private double pos = 0.45;


    @Override
    public void runOpMode() throws InterruptedException {
        boolean[/*L, R*/] wasPressed = {false, false};

        initialize();

        waitForStart();

        while (opModeIsActive()) {

            s.setPosition(pos);
            if(gamepad1.dpad_up && !dpad_up) {pos += increment;dpad_up = true;}
            else dpad_up = false;
            if(gamepad1.dpad_down && !dpad_down) {pos -= increment;dpad_down = true;}
            else dpad_down = false;

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