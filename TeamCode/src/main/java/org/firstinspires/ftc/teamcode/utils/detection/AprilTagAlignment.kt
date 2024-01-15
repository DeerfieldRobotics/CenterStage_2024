package org.firstinspires.ftc.teamcode.utils.detection

import android.util.Size
import com.arcrobotics.ftclib.controller.PIDController
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.utils.hardware.Drivetrain
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
    private val drivetrain: Drivetrain?,
    var targetX: Double,
    var targetY: Double,
    var targetHeading: Double,
){

    var xController = PIDController(0.0174, 0.0, 0.0)
    var yController = PIDController(0.0174, 0.0, 0.0)
    var headingController = PIDController(0.0174, 0.0, 0.0)

    private var aprilTagLibrary = AprilTagLibrary.Builder()
        .addTag(1, "BlueLeft", 2.0, DistanceUnit.INCH)
        .addTag(2, "BlueCenter", 2.0, DistanceUnit.INCH)
        .addTag(3, "BlueRight", 2.0, DistanceUnit.INCH)
        .addTag(4, "RedLeft", 2.0, DistanceUnit.INCH)
        .addTag(5, "RedCenter", 2.0, DistanceUnit.INCH)
        .addTag(6, "RedRight", 2.0, DistanceUnit.INCH)
        .build()

    private val processor = AprilTagProcessorImpl(902.125, 902.125, 604.652, 368.362, DistanceUnit.INCH, AngleUnit.DEGREES, aprilTagLibrary, false, false, false, false, AprilTagProcessor.TagFamily.TAG_36h11, 3) // Used for managing the AprilTag detection process.

    private var visionPortal: VisionPortal = VisionPortal.Builder()
        .setCamera(camera)
        .setCameraResolution(Size(1280, 720))
        .addProcessor(processor)
        .build()

    var targetTagID = -1
    private val tagXOffset = 6.0
    private var detection: AprilTagDetection? = null
    enum class Alliance {
        BLUE, RED
    }
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
        camera: CameraName, drivetrain: Drivetrain?, targetX: Double, targetY: Double,
        targetHeading: Double, xPID: PIDController, yPID: PIDController,
        headingPID: PIDController) : this(camera, drivetrain, targetX, targetY, targetHeading) {
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
        targetFound = false
        detection = null

        val currentDetections: List<AprilTagDetection> = processor.detections
        for (detection in currentDetections) {
            if (targetTagID<0||detection.id == targetTagID) {
                targetFound = true
                this.detection = detection
                break
            }
        }
    }
    fun alignRobot() {
        if (targetFound) {
            xError = targetX - detection!!.ftcPose.x
            yError = targetY - detection!!.ftcPose.y
            headingError = targetHeading - detection!!.ftcPose.yaw

            val xPower = xController.calculate(xError)
            val yPower = yController.calculate(yError)
            val headingPower = headingController.calculate(headingError)

            val forward = forwardMultiplier*(yPower * cos(Math.toRadians(headingError)) + xPower * sin(Math.toRadians(headingError)))
            val strafe = strafeMultiplier*(yPower * sin(Math.toRadians(headingError)) + xPower * cos(Math.toRadians(headingError)))
            val turn = turnMultiplier*headingPower

            drivetrain?.move(forward, strafe, turn)
        }
    }
    fun alignRobotToBackboard(a: Alliance) {
        val rawDetections: List<AprilTagDetection> = processor.detections
        val currentDetections: ArrayList<AprilTagDetection> = ArrayList()
        for(detection in rawDetections)
            if(if(a == Alliance.BLUE) detection.id <= 3 else detection.id >= 4) //Only read the tags corresponding to the alliance
                currentDetections.add(detection)
        var currentX = 0.0
        var currentY = 0.0
        var currentHeading = 0.0
        var total = 0.0
        for (detection in currentDetections) {
            //Weighted average of valid detections weighted by inverse of range squared
            targetFound = true

            currentX += (detection.ftcPose.x - ((detection.id - if(a == Alliance.BLUE) 2 else 5 ) * tagXOffset)) / detection.ftcPose.range.pow(2)
            currentY += detection.ftcPose.y / detection.ftcPose.range.pow(2)
            currentHeading += detection.ftcPose.yaw / detection.ftcPose.range.pow(2)

            total += 1/detection.ftcPose.range.pow(2)
        }

        currentX /= total
        currentY /= total
        currentHeading /= total

        xError = targetX - currentX
        yError = targetY - currentY
        headingError = targetHeading - currentHeading

        val xPower = xController.calculate(xError)
        val yPower = yController.calculate(yError)
        val headingPower = headingController.calculate(headingError)

        val forward = forwardMultiplier*(yPower * cos(Math.toRadians(currentHeading)) + xPower * sin(Math.toRadians(currentHeading)))
        val strafe = strafeMultiplier*(yPower * sin(Math.toRadians(currentHeading)) + xPower * cos(Math.toRadians(currentHeading)))
        val turn = turnMultiplier*headingPower

        drivetrain?.move(forward, strafe, turn)
    }

    fun robotAligned(): Boolean = xController.atSetPoint() && yController.atSetPoint() && headingController.atSetPoint()
}