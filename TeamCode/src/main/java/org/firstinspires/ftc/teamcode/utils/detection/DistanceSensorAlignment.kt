package org.firstinspires.ftc.teamcode.utils.detection

import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.utils.hardware.Drivetrain
import kotlin.math.abs
import kotlin.math.atan
import kotlin.math.cos
import kotlin.math.sin

class DistanceSensorAlignment (hardwareMap: HardwareMap, private val drivetrain: Drivetrain, private var targetDistance: Double, private var targetHeading: Double) {
    private val distanceSensor0:DistanceSensor = hardwareMap.get("ds0") as DistanceSensor //control hub: 0
    private val distanceSensor1:DistanceSensor = hardwareMap.get("ds1") as DistanceSensor //control hub: 1

    private val distanceSensorOffset = .1345 //distance between the two distance sensors in meters

    private val headingPID: PIDController = PIDController(10.0, 0.0, 0.0) //TODO
    private val distancePID: PIDController = PIDController(10.0, 0.0, 0.0) //TODO

    constructor(hardwareMap: HardwareMap, drivetrain: Drivetrain, targetDistance: Double) : this(hardwareMap, drivetrain, targetDistance, 0.0)
    constructor(hardwareMap: HardwareMap, drivetrain: Drivetrain) : this(hardwareMap, drivetrain, 0.5, 0.0)

    init {
        headingPID.setTolerance(5.0)
        distancePID.setTolerance(0.01)
    }
    fun getDistance() = (distanceSensor0.getDistance(DistanceUnit.METER) + distanceSensor1.getDistance(DistanceUnit.METER)) / 2.0

    fun getHeading() = Math.toDegrees(atan((distanceSensor0.getDistance(DistanceUnit.METER) - distanceSensor1.getDistance(DistanceUnit.METER)) / distanceSensorOffset))

    private fun getHeadingError() = targetHeading - getHeading()

    private fun getDistanceError() = targetDistance - getDistance()

    fun driveToTarget() {
        val forwardMultiplier = 1.0 //TODO
        val strafeMultiplier = 1.48 //TODO
        val turnMultiplier = 0.25 //TODO

        val headingError = getHeadingError()
        val distanceError = getDistanceError()
        val headingCorrection = headingPID.calculate(headingError)
        val distanceCorrection = distancePID.calculate(distanceError)
        val forward = forwardMultiplier*distanceCorrection* cos(Math.toRadians(headingError))
        val strafe = strafeMultiplier*distanceCorrection* sin(Math.toRadians(headingError))
        val turn = turnMultiplier*headingCorrection
        val max = abs(forward).coerceAtLeast(abs(strafe).coerceAtLeast(abs(turn)))

        if (max > 1.0) { //if the maximum value is greater than 1, scale down all values by the maximum value such that the proportion of the values is the same
            drivetrain.move(forward / max, strafe / max, turn / max)
        } else
            drivetrain.move(forward, strafe, turn)
    }
}