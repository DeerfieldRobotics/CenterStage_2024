package org.firstinspires.ftc.teamcode.testers;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraName;
import org.firstinspires.ftc.robotcore.internal.camera.names.BuiltinCameraNameImpl;
import org.firstinspires.ftc.teamcode.utils.detection.AprilTagAlignment;

@TeleOp(name = "AprilTagTester for phone", group = "b")
public class AprilTagAlignmentTesterPhone extends LinearOpMode {
    private AprilTagAlignment aprilTagAlignment;
    @Override
    public void runOpMode() throws InterruptedException {
        BuiltinCameraName webcam = BuiltinCameraNameImpl.forCameraDirection(BuiltinCameraDirection.BACK);

        aprilTagAlignment = new AprilTagAlignment(webcam, 0.0, 12.0, 0.0, AprilTagAlignment.Alliance.RED,
            (new PIDController(0.0174, 0.0, 0.0)), //x PID controller
            (new PIDController(0.0174, 0.0, 0.0)), //y PID controller
            (new PIDController(0.0174, 0.0, 0.0))); //heading PID controller

        waitForStart();

        while (opModeIsActive()) {
            aprilTagAlignment.update();

            telemetry.addData("targetTagID", aprilTagAlignment.getTargetTagID());
            telemetry.addData("targetFound", aprilTagAlignment.getTargetFound());
            telemetry.addData("x error","%5.1f inches", aprilTagAlignment.getXError());
            telemetry.addData("y error","%5.1f inches", aprilTagAlignment.getYError());
            telemetry.addData("heading error","%3.0f degrees", aprilTagAlignment.getHeadingError());
            telemetry.addData("Robot Aligned", aprilTagAlignment.robotAligned());
            telemetry.update();
        }
    }
}