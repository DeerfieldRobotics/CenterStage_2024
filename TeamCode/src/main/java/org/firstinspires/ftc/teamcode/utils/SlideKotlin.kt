package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

class SlideKotlin (hardwareMap: HardwareMap){
    private var Slide1: DcMotorEx = hardwareMap.get("sa") as DcMotorEx //expansion hub: 0
    private var Slide2: DcMotorEx = hardwareMap.get("sb") as DcMotorEx //expansion hub: 1
    var minSlideHeight = 1000;
    var targetSlideHeight = 1200;
    var minArmTimeIn = 700;
    var minOuttakeTime = 100;


    init {
        Slide1.direction = DcMotorSimple.Direction.REVERSE
        Slide2.direction = DcMotorSimple.Direction.REVERSE

        Slide1.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        Slide2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        Slide1.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        Slide2.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        Slide1.mode = DcMotor.RunMode.RUN_TO_POSITION
        Slide2.mode = DcMotor.RunMode.RUN_TO_POSITION
    }

    fun setPower (s: Double) {
        Slide1.power = s
        Slide2.power = s
    }
    fun setMode (mode: DcMotor.RunMode) {
        Slide1.mode = mode
        Slide2.mode = mode
    }
    fun setTargetPosition (position: Int) {
        Slide1.targetPosition = position
        Slide2.targetPosition = position
    }
    fun getPosition(): Array<Int> = arrayOf(Slide1.currentPosition, Slide2.currentPosition);
    fun getCurrent(): Array<Double> = arrayOf(Slide1.getCurrent(CurrentUnit.AMPS), Slide2.getCurrent(CurrentUnit.AMPS))
    fun getPower(): Array<Double> = arrayOf(Slide1.power, Slide2.power)
    fun getMode(): Array<DcMotor.RunMode> = arrayOf(Slide1.mode, Slide2.mode)
}