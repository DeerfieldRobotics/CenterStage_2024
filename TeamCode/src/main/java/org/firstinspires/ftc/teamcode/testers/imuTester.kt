package org.firstinspires.ftc.teamcode.testers

import android.util.Log
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.utils.Robot

@TeleOp(name = "IMU Tester", group = "b")
class imuTester: LinearOpMode() {
    private lateinit var robot: Robot
    override fun runOpMode() {
        robot = Robot(hardwareMap)
        waitForStart()
        while(opModeIsActive()) {
            driveNormal()
            telemetry.addData("IMU Angle", robot.getIMUHeading())
            telemetry.update()
            Log.v("IMU", "IMU Angle: ${robot.getIMUHeading()}")
            if(gamepad1.cross || gamepad2.cross) {
                robot.imu.resetYaw()
            }
        }
    }
    private fun driveNormal() {
        //driving values
        val speedMult = .7 + 0.3 * gamepad1.right_trigger - 0.3 * gamepad1.left_trigger
        gamepad1.rumble(
            if (gamepad1.left_trigger > 0.5) (gamepad1.left_trigger - 0.5) / .4 else 0.0,
            if (gamepad1.right_trigger > 0.4) (gamepad1.right_trigger - 0.4) / 0.8 else 0.0,
            50
        )
        val forwardMult = 1.0
        val turnMult = .75
        val strafeMult = 1.0
        val forward = -gamepad1.left_stick_y * forwardMult * speedMult
        val turn = -gamepad1.right_stick_x * turnMult * speedMult
        val strafe = -gamepad1.left_stick_x * strafeMult * speedMult
        robot.drive.setWeightedDrivePower(
            Pose2d(
                forward,
                strafe,
                turn
            )
        )
    }
}