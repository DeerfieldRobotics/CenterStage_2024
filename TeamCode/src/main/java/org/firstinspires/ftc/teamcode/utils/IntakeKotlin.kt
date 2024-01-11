package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.ServoImplEx

class IntakeKotlin(hardwareMap: HardwareMap){
    private var intakeServo: ServoImplEx = hardwareMap.get("is") as ServoImplEx //expansion hub: 0
    private var boosterServo: ServoImplEx = hardwareMap.get("bs") as ServoImplEx //expansion hub: TODO
    private var intakeMotor: DcMotorEx = hardwareMap.get("im") as DcMotorEx  //expansion hub: 2

    private var timeDelayMillis = 0L

    enum class IntakePositions {
        INIT, INTAKE, TRANSFER, FIVE, DRIVE, MANUAL, FOUR, HIGH
    }

    private val intakePositionMap = mapOf( //TODO: find positions
        IntakePositions.INIT to 0.45,
        IntakePositions.INTAKE to 1.0,
        IntakePositions.TRANSFER to 0.67,
        IntakePositions.FIVE to 0.76,
        IntakePositions.FOUR to 0.8,
        IntakePositions.DRIVE to 0.85,
        IntakePositions.HIGH to 0.6
    )

    //Servo variables
    var servoPosition = IntakePositions.INIT
    private var manualServoPosition = 0.0

    //Motor Variables
    var motorMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    var motorTargetPosition = 0
    var motorPower = 0.0
    private val motorPowerMultiplier = 0.75

    enum class TransferStage {
        INIT, TRANSFER, INTAKE, IDLE
    }

    private var transferStage = TransferStage.IDLE

    init {
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        update()
    }

    private fun intakeServo(intakePosition: IntakePositions) {
        if(intakePosition == IntakePositions.MANUAL) {
            intakeServo.position = manualServoPosition
        }
        else
            intakeServo.position = intakePositionMap[intakePosition]!!
        servoPosition = intakePosition
    }

    fun changeIntakeServo(power: Double){
        manualServoPosition = intakeServo.position
        servoPosition = IntakePositions.MANUAL
        manualServoPosition -= power* 0.05
    }

    fun intake (power: Double) {
        if(power > 0.2 && servoPosition != IntakePositions.INTAKE) {
            servoPosition = IntakePositions.INTAKE
        }
        motorPower = power*motorPowerMultiplier
    }

    fun update() {
        transferProcedure()
        intakeMotor.targetPosition = motorTargetPosition
        intakeMotor.mode = motorMode
        intakeServo(servoPosition)
        intakeMotor.power = motorPower
    }

    fun transferProcedure() {
        when(transferStage) {
            TransferStage.INIT -> {
                motorMode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                transferStage = TransferStage.TRANSFER //reset motor
            }
            TransferStage.TRANSFER -> {
                if(timeDelayMillis == 0L) {
                    timeDelayMillis = System.currentTimeMillis()
                }
                servoPosition = IntakePositions.TRANSFER //move servo
                if(System.currentTimeMillis() - timeDelayMillis > 600) { //wait for servo to move to advance
                    transferStage = TransferStage.INTAKE
                    timeDelayMillis = 0L
                }
            }
            TransferStage.INTAKE -> {
                motorMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                motorPower = 1.0 //intake pixels into outtake
                boosterServo.setPosition(1.0) //FIX yo shit
                if(timeDelayMillis == 0L) {
                    timeDelayMillis = System.currentTimeMillis()
                }
                if(System.currentTimeMillis() - timeDelayMillis > 1000) { //wait for pixels to fully intake
                    motorPower = 0.0
                    boosterServo.setPosition(0.0) // FIX
                    transferStage = TransferStage.IDLE
                    timeDelayMillis = 0L
                }
            }
            TransferStage.IDLE -> {
            }
        }
    }

    fun transfer() {
        if(transferStage == TransferStage.IDLE)
            transferStage = TransferStage.INIT
    }

    fun getPosition(): Int = intakeMotor.currentPosition
    fun getIntakeServo(): ServoImplEx = intakeServo
    fun getIntakePos():Double = intakeServo.position
}
