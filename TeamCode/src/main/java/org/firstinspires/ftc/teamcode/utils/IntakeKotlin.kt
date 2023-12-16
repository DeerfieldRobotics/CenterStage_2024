package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.ServoImplEx
import kotlin.math.abs

class IntakeKotlin(hardwareMap: HardwareMap){
    private var intakeServo: ServoImplEx = hardwareMap.get("is") as ServoImplEx //expansion hub: 0
    private var intakeMotor: DcMotorEx = hardwareMap.get("im") as DcMotorEx  //expansion hub: 2

    private var transfer = false
    var transferring = false
    private var currentPosition = IntakePositions.INIT

    private var updateTick = false

    private var t: Thread? = null

    enum class IntakePositions {
        INIT, INTAKE, TRANSFER, FIVE, DRIVE, MANUAL //INIT for init, INTAKE for intaking, TRANSFER for transferring, FIVE for 5 stack, DRIVE for driving, OTHER for custom values
    }
    private val intakePositionMap = mapOf(
            IntakePositions.INIT to 0.5,
            IntakePositions.INTAKE to 1.0,
            IntakePositions.TRANSFER to 0.6994,
            IntakePositions.FIVE to 0.8, //TODO
            IntakePositions.DRIVE to 0.85)

    var motorMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    var servoPosition = IntakePositions.INIT
    var motorTargetPosition = 0
    var motorPower = 0.0
    var motorPosition = 0
    private var motorIsBusy = false
    private var manualPosition = 0.0

    init {
        intakeServo.position = intakePositionMap[servoPosition]!!
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        intakeMotor.mode = motorMode
    }
    private fun intakeServo(intakePosition: IntakePositions) {
        //if switching off of transfer, make sure can switch back
        if(intakePosition != IntakePositions.TRANSFER) {
            transfer = false
        }

        if(intakePosition == IntakePositions.MANUAL) {
            intakeServo.position = manualPosition
        }
        else
            intakeServo.position = intakePositionMap[intakePosition]!!
        currentPosition = intakePosition
    }


    fun changeIntakeServo(power: Double){
        manualPosition = intakeServo.position
        servoPosition = IntakePositions.MANUAL
        manualPosition -= power* 0.05
        transfer = false
    }

    fun intake (power: Double) { //if intaking, make sure the intake is out
        if(abs(power) > 0.2 && servoPosition != IntakePositions.INTAKE) {
            servoPosition = IntakePositions.INTAKE
        }
        motorPower = power
    }
    fun update() {
        if(!transferring) {
            intakeMotor.targetPosition = motorTargetPosition
            intakeMotor.mode = motorMode
            intakeServo(servoPosition)
            intakeMotor.power = motorPower
            motorIsBusy = intakeMotor.isBusy
            motorPosition = intakeMotor.currentPosition
            updateTick = true
        }
    } //griddy griddy on the haters - Charlie Jakymiw 2023

    fun transfer() {
        t?.interrupt() //stops any existing threads
        if (intakeServo.position != intakePositionMap[IntakePositions.TRANSFER]!! && !transfer) {
            transferring = true
            transfer = true
            t?.interrupt() //stops any existing threads
            intakeMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motorMode = DcMotor.RunMode.RUN_TO_POSITION
            intakeMotor.targetPosition = -240 //set value for how much motor needs to outtake to transfer
            motorTargetPosition = -240
            intakeMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
            motorMode = DcMotor.RunMode.RUN_TO_POSITION
            intakeMotor.power = 1.0
            motorPower = 1.0
            t = Thread { //makes a new thread to run the transfer procedure
                while (intakeMotor.isBusy) { //wait for it to finish
                    intakeMotor.power = 1.0
                }
                intakeMotor.power = 0.0
                intakeServo(IntakePositions.TRANSFER)
                var currentTime = System.currentTimeMillis()
                while(System.currentTimeMillis() - currentTime < 400) {

                }
                intakeMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                intakeMotor.power = 1.0
                currentTime = System.currentTimeMillis()
                while (System.currentTimeMillis() - currentTime < 1000) {
                    intakeMotor.power = 1.0
                }
                transfer = true
                transferring = false
                t?.interrupt()
            }
            t?.start()
        }
    }
    fun waitForTick() {
        while (!updateTick) {
        }
        updateTick = false
    }
    fun getPosition(): Int = intakeMotor.currentPosition
    fun getIntakeMotor(): DcMotorEx = intakeMotor
    fun getIntakeServo(): ServoImplEx = intakeServo
    fun getIntakePos():Double = intakeServo.position
}