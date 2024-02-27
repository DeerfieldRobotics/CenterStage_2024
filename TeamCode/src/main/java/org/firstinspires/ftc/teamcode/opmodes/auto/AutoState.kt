package org.firstinspires.ftc.teamcode.opmodes.auto

import android.util.Log
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.utils.Other.LogcatHelper
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.auto.AutoConfigurator
import org.firstinspires.ftc.teamcode.utils.auto.AutoProfile
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.ApriltagPathSegment
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

        telemetry.addData("Selected Auto", PoseHelper.startPosition.toString())
        telemetry.addData("Selected Path", PoseHelper.path.toString())
        telemetry.addData("Detected Position", ColorDetectionProcessor.position.toString())
        telemetry.addData("Front Camera State", robot.frontCameraPortal?.cameraState.toString())
        telemetry.addData("Back Camera State", robot.backCameraPortal?.cameraState.toString())
        telemetry.addData("Front Camera ID", robot.frontCameraID)
        telemetry.addData("Back Camera ID", robot.backCameraID)
        telemetry.update()
        if (robot.backCameraPortal?.cameraState == VisionPortal.CameraState.STREAMING)
            robot.backCameraPortal?.stopStreaming()

        if(isStopRequested) stop()
    }

    override fun runOpMode() {
        initAuto()

        while(!isStarted && !isStopRequested) initLoop()
        waitForStart()

        Log.v(LogcatHelper.TAG, "Detected Position: " + ColorDetectionProcessor.position.toString())

        PoseHelper.buildAuto()

        resetRuntime()
        robot.imu.resetYaw()
//        robot.frontCameraPortal?.close() BROKEN IN 9.1 TODO
        robot.frontCameraPortal?.stopStreaming() //WORKAROUND
        robot.backCameraPortal?.resumeStreaming()

        robot.drive.poseEstimate = PoseHelper.initPose

        for (segment in autoProfile.path) {
            segment.followPathSegment()
            Log.d(LogcatHelper.TAG, "Following Path Segment: $segment")
            while (segment.running && opModeIsActive()) {
                if (segment !is ApriltagPathSegment) //DON'T AUTO LOOP APRILTAG SEGMENTS BECAUSE DRIVE.UPDATE FORCES ROADRUNNER TRAJECTORY
                    autoLoop()
                if (!opModeIsActive() || isStopRequested) break
            }
            if (!opModeIsActive() || isStopRequested) break
        }

        Log.d(LogcatHelper.TAG, "Front Portal Status: "+robot.frontCameraPortal?.cameraState)
        Log.d(LogcatHelper.TAG, "Back Portal Status: "+robot.backCameraPortal?.cameraState)
        Log.d(LogcatHelper.TAG, "Auto Complete")
        stop()
    }

    private fun autoLoop() {
        robot.update()
        Log.v(LogcatHelper.TAG, "robot.drivePose" + robot.drive.poseEstimate)
        Log.v(LogcatHelper.TAG, "robot.intakePosition" + robot.intake.servoPosition)
        Log.v(LogcatHelper.TAG, "robot.outtakePosition" + robot.outtake.outtakePosition)
        Log.v(LogcatHelper.TAG, "robot.slidePosition" + robot.slide.getAvgPosition())
        Log.v(LogcatHelper.TAG, "robot.slideTargetPosition" + robot.slide.getTargetPosition())
        Log.v(LogcatHelper.TAG, "robot.imuHeading "+ (robot.getIMUHeading()))
        Log.v(LogcatHelper.TAG, "robot.frontCameraPortalStatus"+ (robot.frontCameraPortal?.cameraState))
        Log.v(LogcatHelper.TAG, "robot.backCameraPortalStatus"+ (robot.backCameraPortal?.cameraState))
    }

    private fun detectPurplePath() { ColorDetectionProcessor.position = robot.colorDetectionProcessor?.position }
}