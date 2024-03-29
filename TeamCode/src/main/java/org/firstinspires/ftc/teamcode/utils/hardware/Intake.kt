package org.firstinspires.ftc.teamcode.utils.hardware

import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.ServoImplEx

class Intake(hardwareMap: HardwareMap){
    private var intakeServo: ServoImplEx = hardwareMap.get("is") as ServoImplEx //expansion hub: 0
    private var boosterServo: CRServoImplEx = hardwareMap.get("bs") as CRServoImplEx //expansion hub: 4
    private var intakeMotor: DcMotorEx = hardwareMap.get("im") as DcMotorEx  //expansion hub: 2

    var boosterServoPower = 0.0

    private var timeDelayMillis = 0L

    enum class IntakePositions {
        INIT, INTAKE, TRANSFER, FIVE, DRIVE, MANUAL, FOUR, HIGH, THREE, TWO
    }

    private val intakePositionMap = mapOf(
        IntakePositions.INIT to 0.391,
        IntakePositions.INTAKE to 0.0,
        IntakePositions.TRANSFER to 0.293,
        IntakePositions.DRIVE to 0.264,
        IntakePositions.HIGH to 0.352,
        IntakePositions.FIVE to 0.151,
        IntakePositions.FOUR to 0.128,
        IntakePositions.THREE to 0.103,
        IntakePositions.TWO to 0.077
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
        INIT, TRANSFER, INTAKE, IDLE, UNJAM
    }

    private var transferStage = TransferStage.IDLE

    init {
        intakeMotor.direction = DcMotorSimple.Direction.REVERSE
        boosterServo.direction = DcMotorSimple.Direction.REVERSE
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

    fun intakePower (power: Double) {
        motorMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motorPower = power*motorPowerMultiplier
    }

    fun update() {
        transferProcedure()
        intakeMotor.targetPosition = motorTargetPosition
        intakeMotor.mode = motorMode
        intakeServo(servoPosition)
        intakeMotor.power = motorPower
        boosterServo.power = boosterServoPower
    }

    private fun transferProcedure() {
        when(transferStage) {
            TransferStage.INIT -> {
                motorMode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                motorPower = 0.0
                transferStage = TransferStage.TRANSFER //reset motor
            }
            TransferStage.TRANSFER -> {
                if(timeDelayMillis == 0L) {
                    timeDelayMillis = System.currentTimeMillis()
                }
                servoPosition = IntakePositions.TRANSFER //move servo
                if(System.currentTimeMillis() - timeDelayMillis > 600) { //wait for servo to move to advance
                    transferStage = TransferStage.UNJAM
                    timeDelayMillis = 0L
                }
            }
            TransferStage.UNJAM -> {
                if(timeDelayMillis == 0L) {
                    timeDelayMillis = System.currentTimeMillis()
                }
                boosterServoPower = -1.0
                if(System.currentTimeMillis() - timeDelayMillis > 120) {
                    boosterServoPower = 0.0
                    transferStage = TransferStage.INTAKE
                    timeDelayMillis = 0L
                }
            }
            TransferStage.INTAKE -> {
                motorMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                motorPower = 1.0 //intake pixels into outtake
                boosterServoPower = 1.0
                if(timeDelayMillis == 0L) {
                    timeDelayMillis = System.currentTimeMillis()
                }
                if(System.currentTimeMillis() - timeDelayMillis > 1000) { //wait for pixels to fully intake
                    motorPower = 0.0
                    boosterServoPower = 0.0
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
