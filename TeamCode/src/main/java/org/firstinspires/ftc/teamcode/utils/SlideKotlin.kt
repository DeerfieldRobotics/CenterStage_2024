package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

class SlideKotlin (hardwareMap: HardwareMap){
    var Slide1: DcMotorEx = hardwareMap.get("sa") as DcMotorEx
    var Slide2: DcMotorEx = hardwareMap.get("sb") as DcMotorEx


    init {
        Slide1.direction = DcMotorSimple.Direction.REVERSE
        Slide2.direction = DcMotorSimple.Direction.REVERSE

        Slide1.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        Slide2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        Slide1.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        Slide2.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        Slide1.mode = DcMotor.RunMode.RUN_USING_ENCODER
        Slide2.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun setPower (s: Double) {
        Slide1.power = s
        Slide2.power = s
    }
}