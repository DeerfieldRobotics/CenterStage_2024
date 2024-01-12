package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.detection.DistanceSensorAlignment;
import org.firstinspires.ftc.teamcode.utils.hardware.Drivetrain;

@TeleOp(name="Distance Sensor Tester")
public class DistanceSensorTester extends LinearOpMode {
    private Drivetrain drivetrain;
    private DistanceSensorAlignment distanceSensors;
    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new Drivetrain(hardwareMap);
        distanceSensors = new DistanceSensorAlignment(hardwareMap, drivetrain);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Distance", distanceSensors.getDistance());
            telemetry.addData("Distance Error", distanceSensors.getDistance()-distanceSensors.getTargetDistance());
            telemetry.addData("Heading", distanceSensors.getHeading());
            telemetry.addData("Heading Error", distanceSensors.getHeading()-distanceSensors.getTargetHeading());
            telemetry.update();

            if(gamepad1.left_bumper) distanceSensors.driveToTarget();
        }
    }
}
