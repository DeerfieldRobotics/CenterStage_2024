package org.firstinspires.ftc.teamcode.opmodes.auto

import android.util.Log
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.utils.Other.LogcatHelper
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.auto.AutoConfigurator
import org.firstinspires.ftc.teamcode.utils.auto.AutoProfile
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper
import org.firstinspires.ftc.teamcode.utils.detection.AllianceHelper
import org.firstinspires.ftc.teamcode.utils.detection.ColorDetectionProcessor
import org.firstinspires.ftc.teamcode.utils.hardware.Intake
import org.firstinspires.ftc.teamcode.utils.hardware.Outtake
import org.firstinspires.ftc.vision.VisionPortal

@Autonomous(name = "AutoRegionals", preselectTeleOp = "Main Teleop", group = "a")
class AutoState: OpMode() {
    //ROBOT
    private lateinit var robot: Robot

    //CONFIG
    private lateinit var autoConfigurator: AutoConfigurator
    private lateinit var autoProfile: AutoProfile

    override fun init() {
        //SELECT STARTING POSITION
        autoConfigurator = AutoConfigurator(telemetry, gamepad1, gamepad2, robot)
        autoProfile = autoConfigurator.configureAuto()

        //BULK READS
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in allHubs) hub.bulkCachingMode = LynxModule.BulkCachingMode.AUTO

        //DEFINE HARDWARE
        robot = Robot(hardwareMap)

        //SET INITIAL HARDWARE STATES
        robot.outtake.gateClosed = true
        robot.outtake.outtakeProcedureTarget = Outtake.OuttakePositions.INSIDE
        robot.outtake.update()
        robot.intake.servoPosition = Intake.IntakePositions.INIT
        robot.intake.update()

        //INITIALIZE DETECTION PORTALS
        robot.autoInit()
    }

    override fun init_loop() {
        detectPurplePath()
        telemetry.addData("Selected Auto: ", PoseHelper.startPosition.toString())
        telemetry.addData("Selected Path: ", PoseHelper.path.toString())
        telemetry.addData("Detected Position: ", ColorDetectionProcessor.position.toString())
        telemetry.addData("Front Camera State: ", robot.frontCameraPortal?.cameraState.toString())
        telemetry.addData("Back Camera State: ", robot.backCameraPortal?.cameraState.toString())
        telemetry.update()
        if (robot.backCameraPortal?.cameraState == VisionPortal.CameraState.STREAMING)
            robot.backCameraPortal?.stopStreaming()
    }

    override fun start() {
        resetRuntime()
        robot.frontCameraPortal?.stopLiveView()
        robot.backCameraPortal?.resumeStreaming()

        robot.drive.poseEstimate = PoseHelper.initPose

        for (segment in autoProfile.path) {
            segment.followPathSegment()
            while (segment.running) {
                autoLoop()
            }
        }
    }

    override fun loop() {
        autoLoop()
    }
    private fun autoLoop() {
        robot.update()
        Log.v(LogcatHelper.TAG, "robot.drivePose" + robot.drive.poseEstimate)
        Log.v(LogcatHelper.TAG, "robot.intakePosition" + robot.intake.servoPosition)
        Log.v(LogcatHelper.TAG, "robot.outtakePosition" + robot.outtake.outtakePosition)
        Log.v(LogcatHelper.TAG, "robot.outtakeTargetPosition" + robot.outtake.outtakeProcedureTarget)
        Log.v(LogcatHelper.TAG, "robot.slidePosition" + robot.slide.getAvgPosition())
        Log.v(LogcatHelper.TAG, "robot.slideTargetPosition" + robot.slide.getTargetPosition())
    }

    private fun detectPurplePath() {
        ColorDetectionProcessor.position = robot.colorDetectionProcessor?.position
    }
}