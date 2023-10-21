package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

/**
 * Controls the drivetrain of the robot.
 *
 * This class is designed to control a Mecanum drivetrain using four [DcMotorEx] objects. The class requires that the motors be named frontLeft, frontRight, rearLeft, and rearRight accordingly in the configuration of the robot.
 *
 * @author James Xiao
 * @param hardwareMap The hardware map of the robot.
 * @constructor Creates a drivetrain object using 4 [DcMotorEx] objects from the given [HardwareMap].
 */
class DrivetrainKotlin (hardwareMap:HardwareMap) {

    private var frontLeft: DcMotorEx = hardwareMap.get("fl") as DcMotorEx //control hub: 0
    private var frontRight: DcMotorEx = hardwareMap.get("fr") as DcMotorEx //control hub: 1
    private var rearLeft: DcMotorEx = hardwareMap.get("bl") as DcMotorEx //control hub: 2
    private var rearRight: DcMotorEx = hardwareMap.get("br") as DcMotorEx //control hub: 3

    private var imu: IMU = hardwareMap.get("imu") as IMU

    private var powerMultiplier = 1.0

    init {
        frontRight.direction = DcMotorSimple.Direction.REVERSE
        rearLeft.direction = DcMotorSimple.Direction.REVERSE

        frontLeft.mode = DcMotor.RunMode.RUN_USING_ENCODER
        frontRight.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rearLeft.mode = DcMotor.RunMode.RUN_USING_ENCODER
        rearRight.mode = DcMotor.RunMode.RUN_USING_ENCODER

        frontLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        frontRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rearLeft.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rearRight.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE


    }

    /**
     * Moves the drivetrain forward and back using [forward], strafes left and right using [strafe],
     * and turns using [turn]. These parameters should be doubles between -1 and 1 representing a proportion
     * of the full power of the motor.
     */
    fun move(forward: Double, strafe: Double, turn: Double) {
        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        frontLeft.power = powerMultiplier * (forward + turn + strafe)
        frontRight.power = powerMultiplier * (forward - turn - strafe)
        rearLeft.power = powerMultiplier * (forward + turn - strafe)
        rearRight.power = powerMultiplier * (forward - turn + strafe)
    }

    /**
     * Moves the drivetrain forward or back [ticks] amount of ticks.
     */
    fun forward(ticks: Int) {
        setMode(DcMotor.RunMode.RUN_TO_POSITION)
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)

        frontLeft.targetPosition += ticks
        frontRight.targetPosition += ticks
        rearLeft.targetPosition += ticks
        rearRight.targetPosition += ticks

    }

    /**
     * Moves the drivetrain left or right [ticks] amount of ticks.
     */
    fun strafe(ticks: Int) {
        setMode(DcMotor.RunMode.RUN_TO_POSITION)
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)

        frontLeft.targetPosition += ticks
        frontRight.targetPosition -= ticks
        rearLeft.targetPosition -= ticks
        rearRight.targetPosition += ticks
    }

    /**
     * Turns the drivetrain left or right [ticks] amount of ticks.
     */
    fun turn(ticks: Int) {
        setMode(DcMotor.RunMode.RUN_TO_POSITION)
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)

        frontLeft.targetPosition += ticks
        frontRight.targetPosition -= ticks
        rearLeft.targetPosition += ticks
        rearRight.targetPosition -= ticks
    }

    /**
     * Floats the drivetrain by resetting the encoders, setting motor power to zero, and make motors
     * float. The function also makes the motors run using the encoders in order to track the
     * movement.
     */
    fun float() {
        setMotorPower(0.0)
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT)
        setMode(DcMotor.RunMode.RUN_USING_ENCODER)
    }

    /**
     * Sets all of the motor powers to [power]
     */
    fun setMotorPower(power:Double) {
        setMotorPower(power, power, power, power)
    }

    /**
     * Sets each of the motor power values to a given value.
     *
     * @param frontLeftPower The power value for the front left motor.
     * @param frontRightPower The power value for the front right motor.
     * @param rearLeftPower The power value for the rear left motor.
     * @param rearRightPower The power value for the rear right motor.
     *
     */
    fun setMotorPower(frontLeftPower: Double, frontRightPower: Double, rearLeftPower: Double, rearRightPower: Double) {
        frontLeft.power = frontLeftPower
        frontRight.power = frontRightPower
        rearLeft.power = rearLeftPower
        rearRight.power = rearRightPower
    }

    /**
     * Sets the velocity of all motors to [velocity] in ticks per second.
     */
    fun setMotorVelocity(velocity: Double) {
        setMotorVelocity(velocity, velocity, velocity, velocity)
    }

    /**
     * Sets the velocity of all motors to [velocity] in [angleUnit].
     */
    fun setMotorVelocity(velocity: Double, angleUnit: AngleUnit) {
        setMotorVelocity(velocity, velocity, velocity, velocity, angleUnit)
    }

    /**
     * Sets the target velocity of each of the motors in ticks per second.
     *
     * @param frontLeftVelocity The velocity of the front left motor.
     * @param frontRightVelocity The velocity of the front right motor.
     * @param rearLeftVelocity The velocity of the rear left motor.
     * @param rearRightVelocity The velocity of the rear right motor.
     */
    fun setMotorVelocity(frontLeftVelocity: Double, frontRightVelocity: Double, rearLeftVelocity: Double, rearRightVelocity: Double) {
        setMode(DcMotor.RunMode.RUN_USING_ENCODER)

        frontLeft.velocity = frontLeftVelocity
        frontRight.velocity = frontRightVelocity
        rearLeft.velocity = rearLeftVelocity
        rearRight.velocity = rearRightVelocity
    }

    /**
     * Sets the target velocity of each of the motors in the given [AngleUnit].
     *
     * @param frontLeftVelocity The velocity of the front left motor.
     * @param frontRightVelocity The velocity of the front right motor.
     * @param rearLeftVelocity The velocity of the rear left motor.
     * @param rearRightVelocity The velocity of the rear right motor.
     * @param angleUnit The [AngleUnit].
     */
    fun setMotorVelocity(frontLeftVelocity: Double, frontRightVelocity: Double, rearLeftVelocity: Double, rearRightVelocity: Double, angleUnit: AngleUnit) {
        setMode(DcMotor.RunMode.RUN_USING_ENCODER)

        frontLeft.setVelocity(frontLeftVelocity, angleUnit)
        frontRight.setVelocity(frontRightVelocity, angleUnit)
        rearLeft.setVelocity(rearLeftVelocity, angleUnit)
        rearRight.setVelocity(rearRightVelocity, angleUnit)
    }

    /**
     * Uses [behavior] to determine the [DcMotor.ZeroPowerBehavior] of the motors when the power is set to 0.
     */
    fun setZeroPowerBehavior(behavior: DcMotor.ZeroPowerBehavior) {
        frontLeft.zeroPowerBehavior = behavior
        frontRight.zeroPowerBehavior = behavior
        rearLeft.zeroPowerBehavior = behavior
        rearRight.zeroPowerBehavior = behavior
    }

    /**
     * Sets the mode of the motors to [DcMotor.RunMode] [mode]
     */
    fun setMode(mode: DcMotor.RunMode) {
        frontLeft.mode = mode
        frontRight.mode = mode
        rearLeft.mode = mode
        rearRight.mode = mode
    }

    /**
     * Sets the [powerMultiplier] of the drivetrain to [multiplier]. This value should be a double between -1 and 1.
     */
    fun setPowerMultiplier(multiplier: Double) {
        powerMultiplier = multiplier
    }

    /**
     * Returns the [powerMultiplier] of the drivetrain.
     */
    fun getPowerMultiplier(): Double {
        return powerMultiplier
    }

    /**
     * Returns the [DcMotor.RunMode] of all of the motors if they are all equal, otherwise it will return null.
     */
    fun getMode() = if(listOf(frontLeft.mode, frontRight.mode, rearLeft.mode, rearRight.mode).all{it == frontLeft.mode})frontLeft.mode else null

    /**
     * Returns the [DcMotor.ZeroPowerBehavior] ]of all of the motors if they are all equal, otherwise it will return null.
     */
    fun getZeroPowerBehavior() = if(listOf(frontLeft.zeroPowerBehavior, frontRight.zeroPowerBehavior, rearLeft.zeroPowerBehavior, rearRight.zeroPowerBehavior).all{it == frontLeft.zeroPowerBehavior})frontLeft.zeroPowerBehavior else null

    /**
     * Returns an [Array] of encoder positions for all of the motors in the order front left, front right, rear left, rear right.
     */
    fun getEncoderPositions(): Array<Int> = arrayOf(
            frontLeft.currentPosition,
            frontRight.currentPosition,
            rearLeft.currentPosition,
            rearRight.currentPosition)

    /**
     *  Returns an [Array] containing all of the currents of all of the motors in the drivetrain in [CurrentUnit.AMPS]. It is returned in the order, front left, front right, rear left, rear right
     */
    fun getCurrent(): Array<Double> = arrayOf(
            frontLeft.getCurrent(CurrentUnit.AMPS),
            frontRight.getCurrent(CurrentUnit.AMPS),
            rearLeft.getCurrent(CurrentUnit.AMPS),
            rearRight.getCurrent(CurrentUnit.AMPS)
    )

    /**
     *  Returns an [Array] containing all of the velocities of all of the motors in the drivetrain in ticks per second. It is returned in the order, front left, front right, rear left, rear right.
     */
    fun getMotorVelocity(): Array<Double> = arrayOf(frontLeft.velocity, frontRight.velocity, rearLeft.velocity, rearRight.velocity)

    /**
     *  Returns an [Array] containing all of the velocities of all of the motors in the drivetrain in the [AngleUnit] [angleUnit]. It is returned in the order, front left, front right, rear left, rear right.
     */
    fun getMotorVelocity(angleUnit: AngleUnit): Array<Double> = arrayOf(frontLeft.getVelocity(angleUnit), frontRight.getVelocity(angleUnit), rearLeft.getVelocity(angleUnit), rearRight.getVelocity(angleUnit))


    fun getHeading(): Double = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)

    /**
     *  Returns a boolean indicating if any of the motors are busy.
     */
    fun isBusy(): Boolean = frontLeft.isBusy || frontRight.isBusy || rearLeft.isBusy || rearRight.isBusy

    /**
     *  Returns a string containing the power, position, current, and velocity of all of the motors in the drivetrain.
     */
    override fun toString(): String = "frontLeft - power: ${frontLeft.power}, pos: ${frontLeft.currentPosition}, current: ${frontLeft.getCurrent(CurrentUnit.AMPS)}, velocity: ${frontLeft.velocity}\n" +
            "frontRight - power: ${frontRight.power}, pos: ${frontRight.currentPosition}, current: ${frontRight.getCurrent(CurrentUnit.AMPS)}, velocity: ${frontRight.velocity}\n" +
            "rearLeft - power: ${rearLeft.power}, pos: ${rearLeft.currentPosition}, current: ${rearLeft.getCurrent(CurrentUnit.AMPS)}, velocity: ${rearLeft.velocity}\n"+
            "rearRight - power: ${rearRight.power}, pos: ${rearRight.currentPosition}, current: ${rearRight.getCurrent(CurrentUnit.AMPS)}, velocity: ${rearRight.velocity}"
}