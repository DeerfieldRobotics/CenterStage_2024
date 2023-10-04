package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.ServoImplEx

class IntakeKotlin (hardwareMap: HardwareMap){

    private var armServo: ServoImplEx = hardwareMap.get("a") as ServoImplEx
    private var heightServo: ServoImplEx = hardwareMap.get("h") as ServoImplEx
    private var intakeMotor: DcMotorEx = hardwareMap.get("i") as DcMotorEx

    private var armStart: Double = 0.28
    private var armEnd: Double = 1.0
    private var heightStart: Double = 0.0
    private var heightEnd: Double = .8

    init {
        armServo.position = armEnd
        heightServo.position = heightEnd
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        intakeMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    fun arm (g: Double) {
        //maps the grip value from 0 to 1 to actual range
        armServo.position = 1-(g*(armEnd-armStart)+armStart)
    }

    fun height (t: Double) {
        //maps the turn value from 0 to 1 to actual range
        heightServo.position = 1-(t*(heightEnd-heightStart)+heightStart)
    }

    fun intake (p: Double) {
        intakeMotor.power = p
    }
}