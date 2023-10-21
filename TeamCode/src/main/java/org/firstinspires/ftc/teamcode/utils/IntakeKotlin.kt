package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx

class IntakeKotlin (hardwareMap: HardwareMap){


    private var intakeServo: ServoImplEx = hardwareMap.get("is") as ServoImplEx //control hub: 0
    private var outtakeServo: ServoImplEx = hardwareMap.get("os") as ServoImplEx //control hub: 1
    private var armServo: ServoImplEx = hardwareMap.get("as") as ServoImplEx //control hub: 2
    private var intakeMotor: DcMotorEx = hardwareMap.get("im") as DcMotorEx  //expansion hub: 2

    private var intakeStart: Double = 0.0
    private var intakeEnd: Double = 0.0
    private var outtakeStart: Double = 0.0
    private var outtakeEnd: Double = 0.0
    private var armStart: Double = 0.0
    private var armEnd: Double = 0.0

    init {
        intakeServo.position = intakeStart
        outtakeServo.position = outtakeStart
        armServo.position = armStart
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        intakeMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }


    fun intakeServo (position: Double){
        intakeServo.position = position
    }

    fun outtakeServo (position: Double){
        outtakeServo.position = position
    }

    fun armServo(position:Double){
        armServo.position = position;
    }

    fun intake (power: Double) {
        intakeMotor.power = power
    }
}