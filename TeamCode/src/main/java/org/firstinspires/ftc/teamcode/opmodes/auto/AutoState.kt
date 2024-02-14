package org.firstinspires.ftc.teamcode.opmodes.auto

import android.util.Log
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.utils.Other.LogcatHelper
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.auto.AutoConfigurator
import org.firstinspires.ftc.teamcode.utils.auto.AutoProfile
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper
import org.firstinspires.ftc.teamcode.utils.detection.ColorDetectionProcessor
import org.firstinspires.ftc.teamcode.utils.hardware.Intake
import org.firstinspires.ftc.teamcode.utils.hardware.Outtake
import org.firstinspires.ftc.vision.VisionPortal

@Autonomous(name = "AutoState", preselectTeleOp = "Main Teleop", group = "a")
class AutoState: LinearOpMode() {
    //ROBOT
    private lateinit var robot: Robot

    //CONFIG
    private lateinit var autoConfigurator: AutoConfigurator
    private lateinit var autoProfile: AutoProfile

    private fun initAuto() {
        //BULK READS
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in allHubs) hub.bulkCachingMode = LynxModule.BulkCachingMode.AUTO

        //DEFINE HARDWARE
        robot = Robot(hardwareMap)

        //SELECT STARTING POSITION
        autoConfigurator = AutoConfigurator(telemetry, gamepad1, gamepad2, robot)
        autoProfile = autoConfigurator.configureAuto()

        //SET INITIAL HARDWARE STATES
        robot.outtake.gateClosed = true
        robot.outtake.outtakeProcedureTarget = Outtake.OuttakePositions.INSIDE
        robot.outtake.update()
        robot.intake.servoPosition = Intake.IntakePositions.INIT
        robot.intake.update()

        //INITIALIZE DETECTION PORTALS
        robot.autoInit()
    }

    private fun initLoop() {
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

    override fun runOpMode() {
        initAuto()

        while(!isStarted && !isStopRequested) initLoop()
        waitForStart()

        PoseHelper.buildAuto()

        resetRuntime()
        robot.frontCameraPortal?.stopLiveView()
        robot.backCameraPortal?.resumeStreaming()

        robot.drive.poseEstimate = PoseHelper.initPose

        for (segment in autoProfile.path) {
            segment.followPathSegment()
            Log.d(LogcatHelper.TAG, "Following Path Segment: $segment")
            while (segment.running)
                autoLoop()
        }
    }

    private fun autoLoop() {
        robot.update()
        Log.v(LogcatHelper.TAG, "robot.drivePose" + robot.drive.poseEstimate)
        Log.v(LogcatHelper.TAG, "robot.intakePosition" + robot.intake.servoPosition)
        Log.v(LogcatHelper.TAG, "robot.outtakePosition" + robot.outtake.outtakePosition)
        Log.v(LogcatHelper.TAG, "robot.slidePosition" + robot.slide.getAvgPosition())
        Log.v(LogcatHelper.TAG, "robot.slideTargetPosition" + robot.slide.getTargetPosition())
    }

    private fun detectPurplePath() {
        ColorDetectionProcessor.position = robot.colorDetectionProcessor?.position
        Log.v(LogcatHelper.TAG, "Detected Position: " + ColorDetectionProcessor.position.toString())
    }
}