package org.firstinspires.ftc.teamcode.utils.detection

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.controller.PIDController
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.roadrunner.drive.CogchampDrive
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin

class AprilTagAlignmentProcessor(
    private var cameraType: CameraType,
    var targetPose: Pose2d,
    ) : AprilTagProcessorImpl( //FRONT C920 640x480, BACK ARDUCAM 1280x720
    when(cameraType){ CameraType.FRONT -> 622.001; CameraType.BACK -> 903.79 },
    when(cameraType){ CameraType.FRONT -> 622.001; CameraType.BACK -> 903.79 },
    when(cameraType){ CameraType.FRONT -> 319.803; CameraType.BACK -> 620.002 },
    when(cameraType){ CameraType.FRONT -> 241.251; CameraType.BACK -> 360.0 },
    DistanceUnit.INCH,
    AngleUnit.DEGREES,
    AprilTagLibrary.Builder()
        .addTag(1, "BlueLeft", 2.0, DistanceUnit.INCH)
        .addTag(2, "BlueCenter", 2.0, DistanceUnit.INCH)
        .addTag(3, "BlueRight", 2.0, DistanceUnit.INCH)
        .addTag(4, "RedLeft", 2.0, DistanceUnit.INCH)
        .addTag(5, "RedCenter", 2.0, DistanceUnit.INCH)
        .addTag(6, "RedRight", 2.0, DistanceUnit.INCH)
        .addTag(7, "RedLarge", 5.0, DistanceUnit.INCH)
        .addTag(8, "RedSmall", 2.0, DistanceUnit.INCH)
        .addTag(9, "BlueSmall", 2.0, DistanceUnit.INCH)
        .addTag(10, "BlueLarge", 5.0, DistanceUnit.INCH)
        .build(),
    true,
    true,
    true,
    true,
    TagFamily.TAG_36h11,
    3,
        false
) {

    enum class CameraType {
        FRONT, BACK
    }

    private val aprilTagPoseLeftRed: Pose2d = Pose2d(62.0, -29.5, 180.0)
    private val aprilTagPoseCenterRed: Pose2d = Pose2d(62.0, -35.5, 180.0)
    private val aprilTagPoseRightRed: Pose2d = Pose2d(62.0, -41.5, 180.0)
    private val aprilTagPoseLeftBlue: Pose2d = Pose2d(62.0, 41.5, 180.0)
    private val aprilTagPoseCenterBlue: Pose2d = Pose2d(62.0, 35.5, 180.0)
    private val aprilTagPoseRightBlue: Pose2d = Pose2d(62.0, 29.5, 180.0)

    private val aprilTagPoseSmallStackRed: Pose2d = Pose2d(-70.75, -35.47, 180.0)
    private val aprilTagPoseBigStackRed: Pose2d = Pose2d(-70.75, -40.97, 180.0)
    private val aprilTagPoseSmallStackBlue: Pose2d = Pose2d(-70.75, 35.47, 180.0)
    private val aprilTagPoseBigStackBlue: Pose2d = Pose2d(-70.75, 40.97, 180.0)

    //DEFAULT PID VALUES
    var yP = 0.039
    var yI = 0.022
    var yD = 0.0001
    var xP = 0.042
    var xI = 0.038
    var xD = 0.0
    var headingP = 0.820
    var headingI = 0.02
    var headingD = 0.00


    //PID CONTROLLERS
    var yController = PIDController(yP, yI, yD)
    var xController = PIDController(xP, xI, xD)
    var headingController = PIDController(headingP, headingI, headingD)

    //CURRENT POSITIONS
    private var currentY = 0.0
    private var currentX = 0.0
    private var currentHeading = 0.0
    var poseEstimate = Pose2d(currentX, currentY, currentHeading)
    var poseError = Pose2d(0.0, 0.0, 0.0)
        private set

    //PID OUTPUTS
    var yPower = 0.0
        private set
    var xPower = 0.0
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
    private val backCameraOffset = 6.0787402

    private val frontCameraXOffset = -3.7823051181
    private val frontCameraYOffset = -7.625

    var targetFound = false
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

        currentY = 0.0
        currentX = 0.0
        currentHeading = 0.0

        var total = 0.0

        for (detection in detections) {
            if(detection.id == 8 || detection.id == 9)
                break
            //Weighted average of valid detections weighted by inverse of range squared
            targetFound = true

            currentY += (when(detection.id) {
                1 -> aprilTagPoseLeftBlue.y
                2 -> aprilTagPoseCenterBlue.y
                3 -> aprilTagPoseRightBlue.y
                4 -> aprilTagPoseLeftRed.y
                5 -> aprilTagPoseCenterRed.y
                6 -> aprilTagPoseRightRed.y
                7 -> aprilTagPoseBigStackRed.y
                8 -> aprilTagPoseSmallStackRed.y
                9 -> aprilTagPoseSmallStackBlue.y
                10 -> aprilTagPoseBigStackBlue.y
                else -> 0.0
            } + detection.ftcPose.x) / detection.ftcPose.range.pow(2)

            currentX += (when(detection.id) {
                1 -> aprilTagPoseLeftBlue.x
                2 -> aprilTagPoseCenterBlue.x
                3 -> aprilTagPoseRightBlue.x
                4 -> aprilTagPoseLeftRed.x
                5 -> aprilTagPoseCenterRed.x
                6 -> aprilTagPoseRightRed.x
                7 -> aprilTagPoseBigStackRed.x
                8 -> aprilTagPoseSmallStackRed.x
                9 -> aprilTagPoseSmallStackBlue.x
                10 -> aprilTagPoseBigStackBlue.x
                else -> 0.0
            } - detection.ftcPose.y * if(detection.id>=7) -1 else 1) / detection.ftcPose.range.pow(2)

            currentHeading += (when(detection.id) {
                1 -> aprilTagPoseLeftBlue.heading
                2 -> aprilTagPoseCenterBlue.heading
                3 -> aprilTagPoseRightBlue.heading
                4 -> aprilTagPoseLeftRed.heading
                5 -> aprilTagPoseCenterRed.heading
                6 -> aprilTagPoseRightRed.heading
                7 -> aprilTagPoseBigStackRed.heading
                8 -> aprilTagPoseSmallStackRed.heading
                9 -> aprilTagPoseSmallStackBlue.heading
                10 -> aprilTagPoseBigStackBlue.heading
                else -> 0.0
            } - detection.ftcPose.yaw + if((cameraType == CameraType.FRONT && detection.id < 7) || (cameraType == CameraType.BACK && detection.id >= 7)) 180 else 0) / detection.ftcPose.range.pow(2)

            total += 1 / detection.ftcPose.range.pow(2)
        }

        currentY /= total
        currentX /= total
        currentHeading /= total

        when(cameraType) {
            CameraType.BACK -> {
                currentY += sin(Math.toRadians(currentHeading)) * backCameraOffset
                currentX += cos(Math.toRadians(currentHeading)) * backCameraOffset
            }
            CameraType.FRONT -> {
                currentX += cos(Math.toRadians(currentHeading)) * frontCameraYOffset - sin(Math.toRadians(currentHeading)) * frontCameraXOffset
                currentY += sin(Math.toRadians(currentHeading)) * frontCameraYOffset - cos(Math.toRadians(currentHeading)) * frontCameraXOffset
//                currentY += -sin(Math.toRadians(currentHeading)) * frontCameraYOffset - cos(Math.toRadians(currentHeading)) * frontCameraXOffset
//                currentX += -cos(Math.toRadians(currentHeading)) * frontCameraYOffset - sin(Math.toRadians(currentHeading)) * frontCameraXOffset
            }
        }

        if(targetFound) {
            poseEstimate = Pose2d(currentX, currentY, Math.toRadians(currentHeading))
            poseError = targetPose.minus(poseEstimate)
        }
        else {
            poseEstimate = Pose2d(Double.NaN, Double.NaN, Double.NaN)
            poseError = Pose2d(Double.NaN, Double.NaN, Double.NaN)
        }
    }

    fun alignRobot(drivetrain: CogchampDrive?) {
        yPower = 0.0
        xPower = 0.0
        headingPower = 0.0

        forward = 0.0
        strafe = 0.0
        turn = 0.0

        yController.setPID(yP, yI, yD)
        xController.setPID(xP, xI, xD)
        headingController.setPID(headingP, headingI, headingD)

        if(targetFound) {

            yPower = yController.calculate(poseError.y)
            xPower = xController.calculate(poseError.x)
            headingPower = headingController.calculate(poseError.heading)

            forward =
                -forwardMultiplier * (xPower * cos(Math.toRadians(currentHeading)) + yPower * sin(
                    Math.toRadians(currentHeading)
                ))
            strafe =
                -strafeMultiplier * (xPower * sin(Math.toRadians(currentHeading)) + yPower * cos(
                    Math.toRadians(currentHeading)
                ))
            turn = -turnMultiplier * headingPower
        }

        drivetrain?.setWeightedDrivePower(Pose2d(forward, strafe, turn))
    }

    fun robotAligned(): Boolean = poseError.x < 0.5 && poseError.y < 0.5 && poseError.heading < 1.0
}