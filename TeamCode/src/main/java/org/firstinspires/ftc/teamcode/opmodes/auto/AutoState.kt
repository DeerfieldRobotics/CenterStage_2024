package org.firstinspires.ftc.teamcode.opmodes.auto

import android.util.Log
import android.util.Size
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.JavaUtil
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.roadrunner.drive.CogchampDrive
import org.firstinspires.ftc.teamcode.utils.Other.LogcatHelper
import org.firstinspires.ftc.teamcode.utils.auto.AutoConfig
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper
import org.firstinspires.ftc.teamcode.utils.detection.AllianceHelper
import org.firstinspires.ftc.teamcode.utils.detection.AprilTagAlignmentProcessor
import org.firstinspires.ftc.teamcode.utils.detection.ColorDetectionProcessor
import org.firstinspires.ftc.teamcode.utils.hardware.Intake
import org.firstinspires.ftc.teamcode.utils.hardware.Outtake
import org.firstinspires.ftc.teamcode.utils.hardware.Slide
import org.firstinspires.ftc.vision.VisionPortal

@Autonomous(name = "AutoRegionals", preselectTeleOp = "Main Teleop", group = "a")
class AutoState: OpMode() {
    //DETECTION
    private lateinit var colorDetectionProcessor: ColorDetectionProcessor
    private lateinit var aprilTagProcessorBack: AprilTagAlignmentProcessor
    private lateinit var frontCameraPortal: VisionPortal
    private lateinit var backCameraPortal: VisionPortal

    //HARDWARE
    private lateinit var drive: CogchampDrive
    private lateinit var intake: Intake
    private lateinit var outtake: Outtake
    private lateinit var slide: Slide

    //CONFIG
    private lateinit var autoConfig: AutoConfig

    override fun init() {
        //BULK READS
        val allHubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in allHubs) hub.bulkCachingMode = LynxModule.BulkCachingMode.AUTO

        //DEFINE HARDWARE
        drive = CogchampDrive(hardwareMap)
        slide = Slide(hardwareMap)
        intake = Intake(hardwareMap)
        outtake = Outtake(hardwareMap, slide)

        //SET INITIAL HARDWARE STATES
        outtake.gateClosed = true
        outtake.outtakeProcedureTarget = Outtake.OuttakePositions.INSIDE
        outtake.update()
        intake.servoPosition = Intake.IntakePositions.INIT
        intake.update()

        //INITIALIZE DETECTION PORTALS
        initPortals()
    }

    override fun init_loop() {
        detectPurplePath()
        telemetry.addData("Selected Auto: ", PoseHelper.startPosition.toString())
        telemetry.addData("Selected Path: ", PoseHelper.path.toString())
        telemetry.addData("Detected Position: ", ColorDetectionProcessor.position.toString())
        telemetry.addData("Front Camera State: ", frontCameraPortal.cameraState.toString())
        telemetry.addData("Back Camera State: ", backCameraPortal.cameraState.toString())
        telemetry.update()
        if (backCameraPortal.cameraState == VisionPortal.CameraState.STREAMING)
            backCameraPortal.stopStreaming()
    }

    override fun start() {
        resetRuntime()
        frontCameraPortal.stopLiveView()
        backCameraPortal.resumeStreaming()

        drive.poseEstimate = PoseHelper.initPose

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
        drive.update()
        intake.update()
        outtake.update()
        slide.update()
        if (LogcatHelper.verbose) {
            Log.d(LogcatHelper.TAG, "drivePose" + drive.poseEstimate)
            Log.d(LogcatHelper.TAG, "intakePosition" + intake.servoPosition)
            Log.d(LogcatHelper.TAG, "outtakePosition" + outtake.outtakePosition)
            Log.d(LogcatHelper.TAG, "outtakeTargetPosition" + outtake.outtakeProcedureTarget)
            Log.d(LogcatHelper.TAG, "slidePosition" + slide.getAvgPosition())
            Log.d(LogcatHelper.TAG, "slideTargetPosition" + slide.getTargetPosition())
        }
    }
    private fun initPortals() {
        val backCamera: CameraName = hardwareMap.get(WebcamName::class.java, "Webcam 1")
        val frontCamera: CameraName = hardwareMap.get(WebcamName::class.java, "Webcam 2")
        aprilTagProcessorBack = AprilTagAlignmentProcessor(
            AprilTagAlignmentProcessor.CameraType.BACK,
            if (AllianceHelper.alliance == AllianceHelper.Alliance.RED) PoseHelper.backboardCenterRed else PoseHelper.backboardCenterBlue
        ) // Used for managing the april tag detection process.
        colorDetectionProcessor =
            ColorDetectionProcessor(AllianceHelper.alliance) // Used for managing the color detection process.
        val portalList = JavaUtil.makeIntegerList( VisionPortal.makeMultiPortalView( 2, VisionPortal.MultiPortalLayout.HORIZONTAL ) )
        val frontPortalId = JavaUtil.inListGet(portalList, JavaUtil.AtMode.FROM_START, 0, false) as Int
        val backPortalId = JavaUtil.inListGet(portalList, JavaUtil.AtMode.FROM_START, 1, false) as Int
        frontCameraPortal = VisionPortal.Builder()
            .setCamera(frontCamera)
            .setCameraResolution(Size(640, 480))
            .addProcessor(colorDetectionProcessor)
            .setLiveViewContainerId(frontPortalId)
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .build()
        backCameraPortal = VisionPortal.Builder()
            .setCamera(backCamera)
            .setCameraResolution(Size(1280, 720))
            .addProcessor(aprilTagProcessorBack)
            .setLiveViewContainerId(backPortalId)
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .build()
        backCameraPortal.stopLiveView()
    }

    private fun detectPurplePath() {
        ColorDetectionProcessor.position = colorDetectionProcessor.position
    }
    fun selectStartingPosition() {
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