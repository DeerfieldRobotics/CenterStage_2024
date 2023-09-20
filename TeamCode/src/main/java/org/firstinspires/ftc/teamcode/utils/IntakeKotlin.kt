package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.ServoImplEx

class IntakeKotlin (hardwareMap: HardwareMap){

    private var gripServo: ServoImplEx = hardwareMap.get("grip") as ServoImplEx
    private var turnServo: ServoImplEx = hardwareMap.get("turn") as ServoImplEx

    private var gripStart: Double = 0.25
    private var gripEnd: Double = 1.0
    private var turnStart: Double = 0.0
    private var turnEnd: Double = 1.0

    init {
        gripServo.position = gripStart
        turnServo.position = turnStart
    }

    fun grip (g: Double) {
        //maps the grip value from 0 to 1 to actual range
        gripServo.position = g*(gripEnd-gripStart)+gripStart
    }

    fun turn (t: Double) {
        //maps the turn value from 0 to 1 to actual range
        turnServo.position = t*(turnEnd-turnStart)+turnStart
    }
}