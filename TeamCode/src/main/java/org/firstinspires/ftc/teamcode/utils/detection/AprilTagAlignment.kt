package org.firstinspires.ftc.teamcode.utils.detection

import android.util.Size
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.controller.PIDController
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.roadrunner.drive.CogchampDrive
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin

class AprilTagAlignment(
    camera: CameraName,
    var targetX: Double,
    var targetY: Double,
    var targetHeading: Double,
    public var alliance: AllianceHelper.Alliance
){

    var xController = PIDController(0.0174, 0.0, 0.0)
    var yController = PIDController(0.0174, 0.0, 0.0)
    var headingController = PIDController(0.0174, 0.0, 0.0)

    var currentX = 0.0
    var currentY = 0.0
    var currentHeading = 0.0

    var xPower = 0.0
    var yPower = 0.0
    var headingPower = 0.0

    var forward = 0.0
    var strafe = 0.0
    var turn = 0.0

    private val cameraOffset = 6.5;

    private var aprilTagLibrary = AprilTagLibrary.Builder()
        .addTag(1, "BlueLeft", 2.0, DistanceUnit.INCH)
        .addTag(2, "BlueCenter", 2.0, DistanceUnit.INCH)
        .addTag(3, "BlueRight", 2.0, DistanceUnit.INCH)
        .addTag(4, "RedLeft", 2.0, DistanceUnit.INCH)
        .addTag(5, "RedCenter", 2.0, DistanceUnit.INCH)
        .addTag(6, "RedRight", 2.0, DistanceUnit.INCH)
        .build()

    private val processor = AprilTagProcessorImpl(902.125, 902.125, 604.652, 368.362, DistanceUnit.INCH, AngleUnit.DEGREES, aprilTagLibrary, false, false, false, false, AprilTagProcessor.TagFamily.TAG_36h11, 3) // Used for managing the AprilTag detection process.

    private var visionPortal = VisionPortal.Builder()
        .setCamera(camera)
        .setCameraResolution(Size(1280, 720))
        .addProcessor(processor)
        .build()

    var targetTagID = -1
    private val tagXOffset = 6.0 // Lateral offset of tag in inches
    var targetFound = false
        private set

    var xError: Double = 0.0
        private set
    var yError: Double = 0.0
        private set
    var headingError: Double = 0.0
        private set

    private var forwardMultiplier = 1.0
    private var strafeMultiplier = 1.48
    private var turnMultiplier = 0.75
    constructor(
        camera: CameraName, targetX: Double, targetY: Double,
        targetHeading: Double, alliance: AllianceHelper.Alliance, xPID: PIDController, yPID: PIDController,
        headingPID: PIDController) : this(camera, targetX, targetY, targetHeading, alliance) {
        xController = xPID
        yController = yPID
        headingController = headingPID
    }

    init {
        xController.setTolerance(0.5)
        yController.setTolerance(0.5)
        headingController.setTolerance(1.0)
    }

    fun update() {
        val rawDetections: List<AprilTagDetection> = processor.detections
        val currentDetections: ArrayList<AprilTagDetection> = ArrayList()
        for(detection in rawDetections)
            if(if(alliance == AllianceHelper.Alliance.BLUE) detection.id <= 3 else detection.id >= 4) //Only read the tags corresponding to the alliance
                currentDetections.add(detection)
        var total = 0.0
        for (detection in currentDetections) {
            //Weighted average of valid detections weighted by inverse of range squared
            targetFound = true

            currentX += (detection.ftcPose.x - ((detection.id - if(alliance == AllianceHelper.Alliance.BLUE) 2 else 5 ) * tagXOffset)) / detection.ftcPose.range.pow(2)
            currentY += detection.ftcPose.y / detection.ftcPose.range.pow(2)
            currentHeading += detection.ftcPose.yaw / detection.ftcPose.range.pow(2)

            total += 1/detection.ftcPose.range.pow(2)
        }

        currentX /= total
        currentY /= total
        currentHeading /= total

        currentX+=sin(Math.toRadians(currentHeading))*cameraOffset
        currentY+=cos(Math.toRadians(currentHeading))*cameraOffset

        xError = targetX - currentX
        yError = targetY - currentY
        headingError = targetHeading - currentHeading

        xPower = xController.calculate(xError)
        yPower = yController.calculate(yError)
        headingPower = headingController.calculate(headingError)
    }

    fun alignRobotToBackboard(drivetrain: CogchampDrive?) {
        forward = forwardMultiplier*(yPower * cos(Math.toRadians(currentHeading)) + xPower * sin(Math.toRadians(currentHeading)))
        strafe = strafeMultiplier*(yPower * sin(Math.toRadians(currentHeading)) + xPower * cos(Math.toRadians(currentHeading)))
        turn = turnMultiplier*headingPower

        drivetrain?.setWeightedDrivePower(Pose2d(forward, strafe, turn));
    }

    fun robotAligned(): Boolean = xController.atSetPoint() && yController.atSetPoint() && headingController.atSetPoint()
}