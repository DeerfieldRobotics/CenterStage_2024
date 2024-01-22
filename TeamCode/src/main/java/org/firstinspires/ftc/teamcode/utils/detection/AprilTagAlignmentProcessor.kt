package org.firstinspires.ftc.teamcode.utils.detection

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.controller.PIDController
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.roadrunner.drive.CogchampDrive
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin

class AprilTagAlignmentProcessor(
    var cameraType: CameraType,
    var targetX: Double,
    var targetY: Double,
    var targetHeading: Double,
    val alliance: AllianceHelper.Alliance,
    ) : AprilTagProcessorImpl( //BOTH VALUES FOR 1280x720 RESOLUTION, FRONT C920, BACK ARDUCAM
    when(cameraType){ CameraType.FRONT -> 923.95; CameraType.BACK -> 902.125 },
    when(cameraType){ CameraType.FRONT -> 923.95; CameraType.BACK -> 902.125 },
    when(cameraType){ CameraType.FRONT -> 634.655; CameraType.BACK -> 604.652 },
    when(cameraType){ CameraType.FRONT -> 356.056; CameraType.BACK -> 368.362 },
    DistanceUnit.INCH,
    AngleUnit.DEGREES,
    when(cameraType) {
        CameraType.FRONT ->
            AprilTagLibrary.Builder()
                .addTag(7, "RedLarge", 5.0, DistanceUnit.INCH)
                .addTag(8, "RedSmall", 2.0, DistanceUnit.INCH)
                .addTag(9, "BlueSmall", 2.0, DistanceUnit.INCH)
                .addTag(10, "BlueLarge", 5.0, DistanceUnit.INCH)
                .build()
        CameraType.BACK ->
            AprilTagLibrary.Builder()
                .addTag(1, "BlueLeft", 2.0, DistanceUnit.INCH)
                .addTag(2, "BlueCenter", 2.0, DistanceUnit.INCH)
                .addTag(3, "BlueRight", 2.0, DistanceUnit.INCH)
                .addTag(4, "RedLeft", 2.0, DistanceUnit.INCH)
                .addTag(5, "RedCenter", 2.0, DistanceUnit.INCH)
                .addTag(6, "RedRight", 2.0, DistanceUnit.INCH)
                .build() },
    true,
    true,
    true,
    true,
    TagFamily.TAG_36h11,
    3
) {

    enum class CameraType {
        FRONT, BACK
    }

    //DEFAULT PID VALUES
    var xP = 0.04
    var xI = 0.04
    var xD = 0.0006
    var yP = 0.027
    var yI = 0.017
    var yD = 0.0009
    var headingP = 0.03
    var headingI = 0.035
    var headingD = 0.001


    //PID CONTROLLERS
    var xController = PIDController(xP, xI, xD)
    var yController = PIDController(yP, yI, yD)
    var headingController = PIDController(headingP, headingI, headingD)

    //CURRENT POSITIONS
    var currentX = 0.0
        private set
    var currentY = 0.0
        private set
    var currentHeading = 0.0
        private set

    //PID OUTPUTS
    var xPower = 0.0
        private set
    var yPower = 0.0
        private set
    var headingPower = 0.0
        private set

    //DRIVE POWERS
    var forward = 0.0
        private set
    var strafe = 0.0
        private set
    var turn = 0.0
        private set

    //OFFSET OF CAMERA FROM CENTER OF ROTATION
    private val cameraOffset = 6.5 //TODO FIGURE OUT PROPER CAMERA OFFSET, WITH X AN Y COMPONENTS FOR FRONT AND BACK

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

    fun setPIDCoefficients(xP: Double, xI: Double, xD: Double, yP: Double, yI: Double, yD: Double, headingP: Double, headingI: Double, headingD: Double) {
        this.xP = xP
        this.xI = xI
        this.xD = xD
        this.yP = yP
        this.yI = yI
        this.yD = yD
        this.headingP = headingP
        this.headingI = headingI
        this.headingD = headingD
    }

    fun update() {
        targetFound = false

        currentX = 0.0
        currentY = 0.0
        currentHeading = 0.0

        xError = 0.0
        yError = 0.0
        headingError = 0.0

        val rawDetections : List<AprilTagDetection> = detections
        val currentDetections : ArrayList<AprilTagDetection> = ArrayList()

        when(cameraType) {
            CameraType.FRONT -> {
                for (detection in rawDetections)
                    if (if (alliance == AllianceHelper.Alliance.BLUE) detection.id <= 3 else detection.id >= 4) //Only read the tags corresponding to the alliance
                        currentDetections.add(detection)

                var total = 0.0

                for (detection in currentDetections) {
                    //Weighted average of valid detections weighted by inverse of range squared
                    targetFound = true

                    currentX += (detection.ftcPose.x - ((detection.id - if (alliance == AllianceHelper.Alliance.BLUE) 2 else 5) * tagXOffset)) / detection.ftcPose.range.pow(
                        2
                    )
                    currentY += detection.ftcPose.y / detection.ftcPose.range.pow(2)
                    currentHeading += detection.ftcPose.yaw / detection.ftcPose.range.pow(2)

                    total += 1 / detection.ftcPose.range.pow(2)
                }

                currentX /= total
                currentY /= total
                currentHeading /= total

                currentX += sin(Math.toRadians(currentHeading)) * cameraOffset
                currentY += cos(Math.toRadians(currentHeading)) * cameraOffset

                xError = targetX - currentX
                yError = targetY - currentY
                headingError = targetHeading - currentHeading
            }
            CameraType.BACK -> {
                for (detection in rawDetections)
                    if (if (alliance == AllianceHelper.Alliance.BLUE) detection.id >= 9 else detection.id <= 7) //Only read the tags corresponding to the alliance
                        currentDetections.add(detection)

                var total = 0.0

                for (detection in currentDetections) {
                    //Weighted average of valid detections weighted by inverse of range squared
                    targetFound = true

                    currentX += (detection.ftcPose.x + (if(detection.id == 10) 5.5 else if(detection.id == 7) -5.5 else 0.0)) / detection.ftcPose.range.pow(2)
                    currentY += detection.ftcPose.y / detection.ftcPose.range.pow(2)
                    currentHeading += detection.ftcPose.yaw / detection.ftcPose.range.pow(2)

                    total += 1 / detection.ftcPose.range.pow(2)
                }

                currentX /= total
                currentY /= total
                currentHeading /= total

                currentX += sin(Math.toRadians(currentHeading)) * cameraOffset
                currentY += cos(Math.toRadians(currentHeading)) * cameraOffset

                xError = targetX - currentX
                yError = targetY - currentY
                headingError = targetHeading - currentHeading
            }
        }
    }

    fun alignRobotToBackboard(drivetrain: CogchampDrive?) {
        xPower = 0.0
        yPower = 0.0
        headingPower = 0.0

        forward = 0.0
        strafe = 0.0
        turn = 0.0

        xController.setPID(xP, xI, xD)
        yController.setPID(yP, yI, yD)
        headingController.setPID(headingP, headingI, headingD)

        if(targetFound) {

            xPower = xController.calculate(xError)
            yPower = yController.calculate(yError)
            headingPower = headingController.calculate(headingError)

            forward =
                -forwardMultiplier * (yPower * cos(Math.toRadians(currentHeading)) + xPower * sin(
                    Math.toRadians(currentHeading)
                ))
            strafe =
                strafeMultiplier * (yPower * sin(Math.toRadians(currentHeading)) + xPower * cos(
                    Math.toRadians(currentHeading)
                ))
            turn = turnMultiplier * headingPower
        }

        drivetrain?.setWeightedDrivePower(Pose2d(forward, strafe, turn))
    }

    fun robotAligned(): Boolean = xError < 0.5 && yError < 0.5 && headingError < 1.0
}