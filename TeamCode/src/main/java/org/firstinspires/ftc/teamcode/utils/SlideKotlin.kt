package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

class SlideKotlin (hardwareMap: HardwareMap){
    private var slide1: DcMotorEx = hardwareMap.get("sa") as DcMotorEx //expansion hub: 0
    private var slide2: DcMotorEx = hardwareMap.get("sb") as DcMotorEx //expansion hub: 1

    var minSlideHeight = -1000
        private set
    var bottomOut = false
        private set
    var bottomOutProcedure = false

    private var overCurrent = false

    init {
        slide1.direction = DcMotorSimple.Direction.FORWARD
        slide2.direction = DcMotorSimple.Direction.FORWARD

        slide1.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        slide2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        slide1.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        slide2.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        slide1.mode = DcMotor.RunMode.RUN_USING_ENCODER
        slide2.mode = DcMotor.RunMode.RUN_USING_ENCODER

        slide1.setCurrentAlert(6.0, CurrentUnit.AMPS)
        slide2.setCurrentAlert(6.0, CurrentUnit.AMPS)
    }

    fun setPower (s: Double) {
        if(s !=0.0)
            bottomOut = false
        slide1.power = s
        slide2.power = s
    }

    fun setMode (mode: DcMotor.RunMode) {
        slide1.mode = mode
        slide2.mode = mode
    }

    fun setTargetPosition (position: Int) {
        slide1.targetPosition = position
        slide2.targetPosition = position
    }

    fun bottomOut () {
        if (overCurrent) {
            bottomOut = true
            bottomOutProcedure = false
            setPower(0.0)
            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
            setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        }
        if(!bottomOut) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER)
            setPower(1.0)
        }
    }

    private fun checkBottomOut () {
        if ((overCurrent) && (getPosition()[0] > -200 || getPosition()[1] > -200)) {
            bottomOut = true
            bottomOutProcedure = false
            setPower(0.0)
            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
            setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        }
    }

    fun update() {
        overCurrent = slide1.isOverCurrent || slide2.isOverCurrent
        if(bottomOutProcedure && !bottomOut)
            bottomOut()
        else if (bottomOutProcedure)
            bottomOutProcedure = false
        checkBottomOut()
    }

    fun getPosition(): Array<Int> = arrayOf(slide1.currentPosition, slide2.currentPosition)
    private fun getCurrent(): Array<Double> = arrayOf(slide1.getCurrent(CurrentUnit.AMPS), slide2.getCurrent(CurrentUnit.AMPS))
    fun getAvgCurrent(): Double = getCurrent().average()
}
