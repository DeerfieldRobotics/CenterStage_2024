package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.ServoImplEx

class IntakeKotlin (hardwareMap: HardwareMap, private var slide: SlideKotlin){
    private var intakeServo: ServoImplEx = hardwareMap.get("is") as ServoImplEx //control hub: 5
    private var intakeMotor: DcMotorEx = hardwareMap.get("im") as DcMotorEx  //expansion hub: 2

    private var intakeStart: Double = 0.0
    private var intakePositions: Array<Double> = arrayOf(0.05, 0.33, 0.67, 1.0) //array of positions for the intake servo to go to


    init {
        intakeServo.position = intakeStart
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        intakeMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }
    fun intakeServo(position: Int) {
        intakeServo.position = intakePositions[position]
    }


    fun changeIntakeServo(power: Double){
        intakeServo.position -= power* 0.25;
    }

    fun intakeServo (position: Double){
        intakeServo.position = position
    }

    fun intake (power: Double) {
        intakeMotor.power = power
    }

    fun getIntakeMotor(): DcMotorEx = intakeMotor

    fun getIntakeServo(): ServoImplEx = intakeServo
    fun getIntakePos():Double = intakeServo.position
}