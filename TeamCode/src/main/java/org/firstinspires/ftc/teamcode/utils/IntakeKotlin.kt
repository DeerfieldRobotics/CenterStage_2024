package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.ServoImplEx

class IntakeKotlin (hardwareMap: HardwareMap){

    private var gripServo: ServoImplEx = hardwareMap.get("grip") as ServoImplEx
    private var turnServo: ServoImplEx = hardwareMap.get("turn") as ServoImplEx

    fun grip (g: Boolean) {
        if(g)
            gripServo.position = .25
        else
            gripServo.position = 1.0
    }

    fun turn (t: Boolean) {
        if(t)
            turnServo.position = 0.0
        else
            turnServo.position = 1.0
    }
}