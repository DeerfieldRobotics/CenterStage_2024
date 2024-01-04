package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.ServoImplEx

class IntakeKotlin(hardwareMap: HardwareMap){
    private var intakeServo: ServoImplEx = hardwareMap.get("is") as ServoImplEx //expansion hub: 0
    private var intakeMotor: DcMotorEx = hardwareMap.get("im") as DcMotorEx  //expansion hub: 2

    private var currentPosition = IntakePositions.INIT

    private var timeDelayMillis = 0L
    enum class IntakePositions {
        INIT, INTAKE, TRANSFER, FIVE, DRIVE, MANUAL, FOUR, HIGH //INIT for init, INTAKE for intaking, TRANSFER for transferring, FIVE for 5 stack, DRIVE for driving, OTHER for custom values
    }
    private val intakePositionMap = mapOf(
            IntakePositions.INIT to 0.45,
            IntakePositions.INTAKE to 1.0,
            IntakePositions.TRANSFER to 0.67,
            IntakePositions.FIVE to 0.76, //TODO
            IntakePositions.FOUR to 0.8,
            IntakePositions.DRIVE to 0.85,
            IntakePositions.HIGH to 0.6)

    var motorMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    var servoPosition = IntakePositions.INIT
    var motorTargetPosition = 0
    var motorPower = 0.0
    private var motorPosition = 0
    private var motorIsBusy = false
    private var manualPosition = 0.0

    enum class TransferStage {
        INIT, OUTTAKE, TRANSFER, INTAKE, NONE
    }
    private var transferStage = TransferStage.NONE
    init {
        intakeServo.position = intakePositionMap[servoPosition]!!
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        intakeMotor.mode = motorMode
    }
    private fun intakeServo(intakePosition: IntakePositions) {
        //if switching off of transfer, make sure can switch back
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
    }

    fun intake (power: Double) { //if intaking, make sure the intake is out
        if(power > 0.2 && servoPosition != IntakePositions.INTAKE) {
            servoPosition = IntakePositions.INTAKE
        }
        motorPower = power*0.75
    }
    fun update() {
        when(transferStage) {
            TransferStage.INIT -> {
                motorMode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                transferStage = TransferStage.OUTTAKE
            }
            TransferStage.OUTTAKE -> {
                motorTargetPosition = -110
                motorMode = DcMotor.RunMode.RUN_TO_POSITION
                motorPower = 1.0
                if(!motorIsBusy) {
                    transferStage = TransferStage.TRANSFER
                    motorPower = 0.0
                }
            }
            TransferStage.TRANSFER -> {
                if(timeDelayMillis == 0L) {
                    timeDelayMillis = System.currentTimeMillis()
                }
                servoPosition = IntakePositions.TRANSFER
                if(System.currentTimeMillis() - timeDelayMillis > 600) {
                    transferStage = TransferStage.INTAKE
                    timeDelayMillis = 0L
                }
            }
            TransferStage.INTAKE -> {
                motorMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                motorPower = 1.0
                if(timeDelayMillis == 0L) {
                    timeDelayMillis = System.currentTimeMillis()
                }
                if(System.currentTimeMillis() - timeDelayMillis > 1000) {
                    transferStage = TransferStage.NONE
                    timeDelayMillis = 0L
                }
            }
            TransferStage.NONE -> {
            }
        }
        intakeMotor.targetPosition = motorTargetPosition
        intakeMotor.mode = motorMode
        intakeServo(servoPosition)
        intakeMotor.power = motorPower
        motorIsBusy = intakeMotor.isBusy
        motorPosition = intakeMotor.currentPosition
    }
    fun transfer() {
        transferStage = TransferStage.INIT
    }
    fun getPosition(): Int = intakeMotor.currentPosition
    fun getIntakeServo(): ServoImplEx = intakeServo
    fun getIntakePos():Double = intakeServo.position
}