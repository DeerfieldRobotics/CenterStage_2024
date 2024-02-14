package org.firstinspires.ftc.teamcode.opmodes.auto

import android.util.Log
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.utils.Other.LogcatHelper
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.auto.AutoConfig
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
    private lateinit var autoConfig: AutoConfig

    override fun init() {
        //SELECT STARTING POSITION
        selectStartingPosition()

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

        for (segment in autoConfig.path) {
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
    private fun selectStartingPosition() {
        while (true) {
            telemetry.addLine("             [15118 AUTO INITIALIZED]")
            telemetry.addLine("-------------------------------------------------")
            telemetry.addLine("Select Autonomous Starting Position using DPAD Keys")
            telemetry.addData("    Blue Close   ", "(^)")
            telemetry.addData("    Blue Far     ", "(v)")
            telemetry.addData("    Red Far      ", "(<)")
            telemetry.addData("    Red Close    ", "(>)")
            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                PoseHelper.startPosition = PoseHelper.StartPosition.BLUE_CLOSE
                AllianceHelper.alliance = AllianceHelper.Alliance.BLUE
                break
            }
            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                PoseHelper.startPosition = PoseHelper.StartPosition.BLUE_FAR
                AllianceHelper.alliance = AllianceHelper.Alliance.BLUE
                break
            }
            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                PoseHelper.startPosition = PoseHelper.StartPosition.RED_FAR
                AllianceHelper.alliance = AllianceHelper.Alliance.RED
                break
            }
            if (gamepad1.dpad_right || gamepad2.dpad_right) {
                PoseHelper.startPosition = PoseHelper.StartPosition.RED_CLOSE
                AllianceHelper.alliance = AllianceHelper.Alliance.RED
                break
            }
            telemetry.update()
        }
        while (true) {
            telemetry.addLine("             [15118 AUTO INITIALIZED]")
            telemetry.addLine("-------------------------------------------------")
            telemetry.addLine(" Selected " + PoseHelper.startPosition.toString() + " Starting Position.")
            telemetry.addLine()
            telemetry.addLine("Select Autonomous Path using Shape Buttons")
            telemetry.addData("     Inside      ", "(Triangle)")
            telemetry.addData("     Outside     ", "(Cross)")
            telemetry.addData("     Placement   ", "(Circle)")
            telemetry.addLine()
            telemetry.addLine("Select Logging Level using Bumper Buttons")
            telemetry.addData(
                if (LogcatHelper.verbose) "-----verbose-----" else "     verbose     ",
                "(L2)"
            )
            telemetry.addData(
                if (!LogcatHelper.verbose) "-----normal------" else "     normal      ",
                "(R2)"
            )
            if (gamepad1.triangle || gamepad2.triangle) {
                PoseHelper.path = PoseHelper.Path.INSIDE
                break
            }
            if (gamepad1.cross || gamepad2.cross) {
                PoseHelper.path = PoseHelper.Path.OUTSIDE
                break
            }
            if (gamepad1.circle || gamepad2.circle) {
                PoseHelper.path = PoseHelper.Path.PLACEMENT
                break
            }
            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                LogcatHelper.verbose = true
            }
            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                LogcatHelper.verbose = false
            }
            telemetry.update()
        }
            //TODO add configs
        telemetry.clear()
    }
}