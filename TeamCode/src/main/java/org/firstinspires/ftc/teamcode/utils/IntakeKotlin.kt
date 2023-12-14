package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.ServoImplEx
import kotlin.math.abs

class IntakeKotlin(hardwareMap: HardwareMap){
    private var intakeServo: ServoImplEx = hardwareMap.get("is") as ServoImplEx //control hub: 5
    private var intakeMotor: DcMotorEx = hardwareMap.get("im") as DcMotorEx  //expansion hub: 2

    private var transfer = false
    private var currentPosition = IntakePositions.INIT

    private var t: Thread? = null

    enum class IntakePositions {
        INIT, INTAKE, TRANSFER, FIVE, DRIVE, MANUAL //INIT for init, INTAKE for intaking, TRANSFER for transferring, FIVE for 5 stack, DRIVE for driving, OTHER for custom values
    }
    private val intakePositionMap = mapOf(
            IntakePositions.INIT to 0.4,
            IntakePositions.INTAKE to 1.0,
            IntakePositions.TRANSFER to 0.6994,
            IntakePositions.FIVE to 0.8, //TODO
            IntakePositions.DRIVE to 0.85)

    private var motorMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    private var servoPosition = IntakePositions.INIT
    private var motorTargetPosition = 0
    private var motorPower = 0.0
    private var motorIsBusy = false
    private var manualPosition = 0.0

    init {
        intakeServo.position = intakePositionMap[servoPosition]!!
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        intakeMotor.mode = motorMode
    }
    fun intakeServo(intakePosition: IntakePositions) {
        //if switching off of transfer, make sure can switch back
        if(intakePosition != IntakePositions.TRANSFER)
            transfer = false
        if(intakePosition == IntakePositions.MANUAL) {
            intakeServo.position = manualPosition
            return
        }
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
            intakeServo(IntakePositions.INTAKE)
        }
        motorPower = power
    }
    fun update() {
        intakeMotor.targetPosition = motorTargetPosition
        intakeMotor.mode = motorMode
        intakeServo(servoPosition)
        intakeMotor.power = motorPower
        motorIsBusy = intakeMotor.isBusy
    } //griddy griddy on the haters - Charlie Jakymiw 2023
    fun transfer() {
        if (intakeServo.position != intakePositionMap[IntakePositions.TRANSFER]!! && !transfer) {
            t?.interrupt() //stops any existing threads
            t = Thread { //makes a new thread to run the transfer procedure
                try {
                    motorMode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                    intakeMotor.targetPosition = -240 //set value for how much motor needs to outtake to transfer

                    motorMode = DcMotor.RunMode.RUN_TO_POSITION
                    motorPower = 0.8
                    while (motorIsBusy) { //wait for it to finish
                        motorPower = 0.8
                    }
                    servoPosition = IntakePositions.TRANSFER
                    Thread.sleep(400)
                    motorMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                    transfer = true
                    val currentTime = System.currentTimeMillis()
                    while (System.currentTimeMillis() - currentTime < 500) {
                        motorPower = 1.0
                    }
                } catch (e: InterruptedException) {
                    Thread.currentThread().interrupt()
                }
            }
            t!!.start()
        }
        else if (transfer) {
            motorPower = 1.0
        }

    }
    fun getPosition(): Int = intakeMotor.currentPosition
    fun getIntakeMotor(): DcMotorEx = intakeMotor
    fun getIntakeServo(): ServoImplEx = intakeServo
    fun getIntakePos():Double = intakeServo.position
}