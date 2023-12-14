package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.ServoImplEx
import kotlin.math.abs

class IntakeKotlin(hardwareMap: HardwareMap){
    private var intakeServo: ServoImplEx = hardwareMap.get("is") as ServoImplEx //control hub: 5
    private var intakeMotor: DcMotorEx = hardwareMap.get("im") as DcMotorEx  //expansion hub: 2

    private var transfer = false
    var currentPosition = IntakePositions.INIT

    private var t: Thread? = null

    enum class IntakePositions {
        INIT, INTAKE, TRANSFER, FIVE, DRIVE //INIT for init, INTAKE for intaking, TRANSFER for transferring, FIVE for 5 stack, DRIVE for driving
    }
    private val intakePositionMap = mapOf(
            IntakePositions.INIT to 0.4,
            IntakePositions.INTAKE to 1.0,
            IntakePositions.TRANSFER to 0.6994,
            IntakePositions.FIVE to 0.8, //TODO
            IntakePositions.DRIVE to 0.85)

    init {
        intakeServo.position = intakePositionMap[IntakePositions.INIT]!!
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        intakeMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }
    fun intakeServo(intakePosition: IntakePositions) {
        //if switching off of transfer, make sure can switch back
        if(intakePosition != IntakePositions.TRANSFER)
            transfer = false
        intakeServo.position = intakePositionMap[intakePosition]!!
        currentPosition = intakePosition
    }


    fun changeIntakeServo(power: Double){
        intakeServo.position -= power* 0.05
        transfer = false
    }

    fun intake (power: Double) { //if intaking, make sure the intake is out
        if(abs(power) > 0.2 && intakeServo.position != intakePositionMap[IntakePositions.INTAKE]!!) {
            intakeServo(IntakePositions.INTAKE)
        }
        intakeMotor.power = power
    }
    fun transfer() {
        if (intakeServo.position != intakePositionMap[IntakePositions.TRANSFER]!! && !transfer) {
            t?.interrupt() //stops any existing threads
            t = Thread { //makes a new thread to run the transfer procedure
                try {
                    intakeMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                    intakeMotor.targetPosition = -240 //set value for how much motor needs to outtake to transfer

                    val pidfCoefficients = PIDFCoefficients(30.0, 3.0, 0.0, 0.0, MotorControlAlgorithm.LegacyPID)
                    intakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients)

                    intakeMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                    intakeMotor.power = 0.8
                    while (intakeMotor.isBusy) { //wait for it to finish
                        intakeMotor.power = 0.8
                    }
                    intakeServo(IntakePositions.TRANSFER)
                    Thread.sleep(400)
                    intakeMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                    transfer = true
                    val currentTime = System.currentTimeMillis()
                    while (System.currentTimeMillis() - currentTime < 500) {
                        intakeMotor.power = 1.0
                    }
                } catch (e: InterruptedException) {
                    Thread.currentThread().interrupt()
                }
            }
            t!!.start()
        }
        else if (transfer) {
            intakeMotor.power = 1.0
        }

    }
    fun getPosition(): Int = intakeMotor.currentPosition
    fun getIntakeMotor(): DcMotorEx = intakeMotor
    fun getIntakeServo(): ServoImplEx = intakeServo
    fun getIntakePos():Double = intakeServo.position
}