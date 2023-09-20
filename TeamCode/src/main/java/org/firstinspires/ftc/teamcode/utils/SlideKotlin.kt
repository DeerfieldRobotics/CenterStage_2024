package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

class SlideKotlin (hardwareMap: HardwareMap){
    var Slide: DcMotorEx = hardwareMap.get("s") as DcMotorEx

    init {
        Slide.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        Slide.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        Slide.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun setPower (s: Double) {
        Slide.power = s
    }
    fun getCurrent (): Double {
        return Slide.getCurrent(CurrentUnit.AMPS)
    }
}