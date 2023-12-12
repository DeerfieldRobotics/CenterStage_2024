package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.ServoImplEx

class IntakeKotlin (hardwareMap: HardwareMap, private var slide: SlideKotlin){
    private var intakeServo: ServoImplEx = hardwareMap.get("is") as ServoImplEx //control hub: 5
    private var intakeMotor: DcMotorEx = hardwareMap.get("im") as DcMotorEx  //expansion hub: 2

    private var intakeStart: Double = 1.0

    private var transfer = false

    private var t: Thread? = null

    enum class IntakePositions {
        IN, OUT, TRANSFER, FIVE, DRIVE //IN FOR INIT POSITION, OUT FOR REGULAR POSITION, SLIDE FOR TRANSFER POSITION, FIVE FOR FIVE STACK POSITION
    }
    private val intakePositionMap = mapOf(
            IntakePositions.IN to 0.2994,
            IntakePositions.OUT to 1.0,
            IntakePositions.TRANSFER to 0.6994,
            IntakePositions.FIVE to 0.8, //TODO
            IntakePositions.DRIVE to 0.9)

    init {
        intakeServo.position = intakeStart
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        intakeMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }
    fun intakeServo(intakePosition: IntakePositions) {
        //if switching off of transfer, make sure can switch back
        if(intakePosition != IntakePositions.TRANSFER && intakeServo.position == intakePositionMap[IntakePositions.TRANSFER]!!)
            transfer = false
        intakeServo.position = intakePositionMap[intakePosition]!!
    }


    fun changeIntakeServo(power: Double){
        intakeServo.position -= power* 0.05
        transfer = false
    }

    fun intake (power: Double) { //if intaking, make sure the intake is out
        if(power != 0.0)
            intakeServo(IntakePositions.OUT)
        intakeMotor.power = power
    }
    fun transfer() {
        if (intakeServo.position != intakePositionMap[IntakePositions.TRANSFER]!! && !transfer) {
            t?.interrupt() //stops any existing threads
            t = Thread { //makes a new thread to run the transfer procedure
                try {
                    intakeMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                    intakeMotor.targetPosition = -250 //set value for how much motor needs to outtake to transfer

                    val pidfCoefficients = PIDFCoefficients(30.0, 3.0, 0.0, 0.0, MotorControlAlgorithm.LegacyPID)
                    intakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfCoefficients)

                    intakeMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                    intakeMotor.power = 1.0
                    while (intakeMotor.isBusy) { //wait for it to finish
                        intakeMotor.power = 1.0
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
    fun getPosition(): Int {
        return intakeMotor.currentPosition
    }
    fun getIntakeMotor(): DcMotorEx = intakeMotor

    fun getIntakeServo(): ServoImplEx = intakeServo
    fun getIntakePos():Double = intakeServo.position
}