package org.firstinspires.ftc.teamcode.testers;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.detection.AprilTagAlignment;
import org.firstinspires.ftc.teamcode.utils.hardware.Drivetrain;

import java.util.Arrays;
import java.util.Collections;

@TeleOp(name = "AprilTagTester")
public class AprilTagAlignmentTester extends LinearOpMode {
    private Drivetrain drivetrain;
    private AprilTagAlignment aprilTagAlignment;

    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain = new Drivetrain(hardwareMap);
        aprilTagAlignment = new AprilTagAlignment(hardwareMap, drivetrain, 0.0, 12.0, 0.0,
            (new PIDController(0.0174, 0.0, 0.0)), //x PID controller
            (new PIDController(0.0174, 0.0, 0.0)), //y PID controller
            (new PIDController(0.0174, 0.0, 0.0))); //heading PID controller

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.left_bumper && aprilTagAlignment.getTargetFound()) {
                aprilTagAlignment.update();
            }
            else
                driveNormal();

            telemetry.addData("targetFound", aprilTagAlignment.getTargetFound());
            telemetry.addData("x error","%5.1f inches", aprilTagAlignment.getXError());
            telemetry.addData("y error","%5.1f inches", aprilTagAlignment.getYError());
            telemetry.addData("heading error","%3.0f degrees", aprilTagAlignment.getHeadingError());
            telemetry.addData("drivetrain power", Collections.max(Arrays.asList(drivetrain.getMotorPower())));
            telemetry.update();
        }
    }
    private void driveNormal() {
        double speedMult = .7+0.3 * gamepad1.right_trigger-0.3*gamepad1.left_trigger;

        gamepad1.rumble(gamepad1.left_trigger>0.5?(gamepad1.left_trigger-0.5)/.4:0.0,gamepad1.right_trigger>0.4?(gamepad1.right_trigger-0.4)/0.8:0.0,50);

        double forwardMult = 1;
        double turnMult = .75;
        double strafeMult = 1;

        double forward = gamepad1.left_stick_y * forwardMult * speedMult;
        double turn = -gamepad1.right_stick_x * turnMult * speedMult;
        double strafe = -gamepad1.left_stick_x * strafeMult * speedMult;

        drivetrain.move(forward, strafe, turn);
    }
}

