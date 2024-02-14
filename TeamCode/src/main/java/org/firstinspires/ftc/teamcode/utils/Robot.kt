package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.JavaUtil
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.roadrunner.drive.CogchampDrive
import org.firstinspires.ftc.teamcode.utils.detection.AllianceHelper
import org.firstinspires.ftc.teamcode.utils.detection.AprilTagAlignmentProcessor
import org.firstinspires.ftc.teamcode.utils.detection.ColorDetectionProcessor
import org.firstinspires.ftc.teamcode.utils.hardware.Intake
import org.firstinspires.ftc.teamcode.utils.hardware.Outtake
import org.firstinspires.ftc.teamcode.utils.hardware.Slide
import org.firstinspires.ftc.vision.VisionPortal

class Robot (hardwareMap: HardwareMap) {
    val intake = Intake(hardwareMap)
    val slide = Slide(hardwareMap)
    val outtake = Outtake(hardwareMap, slide)
    val drive = CogchampDrive(hardwareMap)

    private val backCamera: CameraName = hardwareMap.get(WebcamName::class.java, "Webcam 1")
    private val frontCamera: CameraName = hardwareMap.get(WebcamName::class.java, "Webcam 2")

    var aprilTagProcessorBack: AprilTagAlignmentProcessor? = null
    var colorDetectionProcessor: ColorDetectionProcessor? = null

    var frontCameraPortal: VisionPortal? = null
    var backCameraPortal: VisionPortal? = null

    fun autoInit() {

        aprilTagProcessorBack = AprilTagAlignmentProcessor(
            AprilTagAlignmentProcessor.CameraType.BACK,
            if (AllianceHelper.alliance == AllianceHelper.Alliance.RED) org.firstinspires.ftc.teamcode.utils.auto.PoseHelper.backboardCenterRed else org.firstinspires.ftc.teamcode.utils.auto.PoseHelper.backboardCenterBlue
        ) // Used for managing the april tag detection process.
        colorDetectionProcessor =
            ColorDetectionProcessor(AllianceHelper.alliance) // Used for managing the color detection process.

        val portalList = JavaUtil.makeIntegerList(
            VisionPortal.makeMultiPortalView(
                2,
                VisionPortal.MultiPortalLayout.HORIZONTAL
            )
        )

        val frontPortalId =
            JavaUtil.inListGet(portalList, JavaUtil.AtMode.FROM_START, 0, false) as Int
        val backPortalId =
            JavaUtil.inListGet(portalList, JavaUtil.AtMode.FROM_START, 1, false) as Int
        frontCameraPortal = VisionPortal.Builder()
            .setCamera(frontCamera)
            .setCameraResolution(android.util.Size(640, 480))
            .addProcessor(colorDetectionProcessor)
            .setLiveViewContainerId(frontPortalId)
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .build()
        backCameraPortal = VisionPortal.Builder()
            .setCamera(backCamera)
            .setCameraResolution(android.util.Size(1280, 720))
            .addProcessor(aprilTagProcessorBack)
            .setLiveViewContainerId(backPortalId)
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .build()
        backCameraPortal?.stopLiveView()
    }

    fun update() {
        drive.update()
        intake.update()
        outtake.update()
        slide.update()
    }
}