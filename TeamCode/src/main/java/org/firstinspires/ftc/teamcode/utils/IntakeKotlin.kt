package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.ServoImplEx

class IntakeKotlin (hardwareMap: HardwareMap, private var slide: SlideKotlin){
    private var intakeServo: ServoImplEx = hardwareMap.get("is") as ServoImplEx //control hub: 5
    private var intakeMotor: DcMotorEx = hardwareMap.get("im") as DcMotorEx  //expansion hub: 2

    private var intakeStart: Double = 1.0

    private var t: Thread? = null

    enum class IntakePositions {
        IN, OUT, SLIDE, FIVE //IN FOR INIT POSITION, OUT FOR REGULAR POSITION, SLIDE FOR TRANSFER POSITION, FIVE FOR FIVE STACK POSITION
    }
    private val IntakePositionMap = mapOf(
            IntakePositions.IN to 0.2994,
            IntakePositions.OUT to 1.0,
            IntakePositions.SLIDE to 0.6994,
            IntakePositions.FIVE to 0.8) //TODO

    init {
        intakeServo.position = intakeStart
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        intakeMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        intakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, PIDFCoefficients(30.0, 3.0, 0.0, 0.0, MotorControlAlgorithm.LegacyPID))
    }
    fun intakeServo(intakePosition: IntakePositions) {
        intakeServo.position = IntakePositionMap[intakePosition]!!
    }


    fun changeIntakeServo(power: Double){
        intakeServo.position -= power* 0.05;
    }

    fun intakeServo (position: Double){
        intakeServo.position = position
    }

    fun intake (power: Double) {
        intakeMotor.power = power
    }
    fun jig() {

        t?.interrupt() //stops any existing threads
        t = Thread { //makes a new thread to run the outtake procedure
            try {
                intakeMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                intakeMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                intakeMotor.targetPosition = -270
                intakeMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                intakeMotor.power = 0.8
                while (intakeMotor.isBusy) {
                    intakeMotor.power = 0.8
                }
                intakeServo(IntakePositions.SLIDE)
                Thread.sleep(500)
                intakeMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                intakeMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
                val currentTime = System.currentTimeMillis()
                while(System.currentTimeMillis() - currentTime < 800) {
                    intakeMotor.power = 0.8
                }
            } catch (e: InterruptedException) {
                Thread.currentThread().interrupt()
            }
        }
        t!!.start()

    }
    fun getPosition(): Int {
        return intakeMotor.currentPosition
    }
    fun getIntakeMotor(): DcMotorEx = intakeMotor

    fun getIntakeServo(): ServoImplEx = intakeServo
    fun getIntakePos():Double = intakeServo.position
}