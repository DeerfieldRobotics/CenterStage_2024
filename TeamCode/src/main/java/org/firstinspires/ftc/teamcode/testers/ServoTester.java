package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "Servo Tester", group = "a")
public class ServoTester extends LinearOpMode {
    private Servo s;
    private double maxSpeed = 0.001; // Adjust this value to change the maximum speed
    private double servoPosition = 0.5; // Initial position of the servo

    @Override
    public void runOpMode() throws InterruptedException {
        s = hardwareMap.get(Servo.class, "as");

        waitForStart();

        while (opModeIsActive()) {
            double stickPosition = -gamepad2.right_stick_y; // Negate to make up increase position
            servoPosition += maxSpeed * stickPosition; // Increment position based on stick movement

            // Ensure servoPosition is within [0, 1]
            if (servoPosition > 1) {
                servoPosition = 1;
            } else if (servoPosition < 0) {
                servoPosition = 0;
            }

            s.setPosition(servoPosition);

            telemetry.addData("Servo Position", s.getPosition());
            telemetry.update();
        }
    }
}