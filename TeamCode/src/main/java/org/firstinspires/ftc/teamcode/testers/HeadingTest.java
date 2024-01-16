package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Heading Tester", group = "c")
@Disabled
public class HeadingTest extends LinearOpMode {
    private IMU imu;
    // TODO fix parameters for c-hub placement
    private IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP
    ));

    @Override
    public void runOpMode() {
        initialize();

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.ps) { imu.resetYaw(); }

            double h = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            telemetry.addData("Heading (deg)", Math.toDegrees(h));
            telemetry.addData("Heading (rad)", h);

            telemetry.update();
        }

    }

    public void initialize() {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);
    }

}