package org.firstinspires.ftc.teamcode.testers;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "servotester")
public class ServoTest extends LinearOpMode {
    private Servo s;
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        s = hardwareMap.get(Servo.class, "servo");

        waitForStart();
        while(opModeIsActive()) {
            s.setPosition(.4);
            telemetry.addData("POSITION", s.getPosition());
            telemetry.update();
            Thread.sleep(5000);
            s.setPosition(1);
            Thread.sleep(1000);
            telemetry.addData("POSITION", s.getPosition());
            telemetry.update();
        }
    }

    public void initialize() {

    }
}