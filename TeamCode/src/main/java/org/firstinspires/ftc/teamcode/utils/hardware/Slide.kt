package org.firstinspires.ftc.teamcode.utils.hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

class Slide (hardwareMap: HardwareMap){
    private var slide1: DcMotorEx = hardwareMap.get("sa") as DcMotorEx //expansion hub: 0
    private var slide2: DcMotorEx = hardwareMap.get("sb") as DcMotorEx //expansion hub: 1

    var minSlideHeight = -800
        private set
    var bottomOut = false
        private set
    var bottomOutProcedure = false

    var power = 0.0
        set(value) {
            field = value
            if (value != 0.0) {
                bottomOut = false
            }
        }

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

        slide1.setCurrentAlert(8.0, CurrentUnit.AMPS)
        slide2.setCurrentAlert(8.0, CurrentUnit.AMPS)
    }

    fun setMode (mode: DcMotor.RunMode) {
        slide1.mode = mode
        slide2.mode = mode
    }

    fun setTargetPosition (position: Int) {
        slide1.targetPosition = position
        slide2.targetPosition = position
    }

    private fun bottomOut () {
        if (overCurrent) {
            bottomOut = true
            bottomOutProcedure = false
            power = 0.0
            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
            setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        }
        if(!bottomOut) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER)
            power = 1.0
        }
    }

    private fun checkBottomOut () {
        if ((overCurrent) && (getPosition()[0] > -200 || getPosition()[1] > -200)) {
            bottomOut = true
            bottomOutProcedure = false
            power = 0.0
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
        slide1.power = power
        slide2.power = power
    }

    fun getPosition(): Array<Int> = arrayOf(slide1.currentPosition, slide2.currentPosition)
    fun getAvgPosition(): Int = getPosition().average().toInt()
    fun getTargetPosition(): Int = slide1.targetPosition
    private fun getCurrent(): Array<Double> = arrayOf(slide1.getCurrent(CurrentUnit.AMPS), slide2.getCurrent(CurrentUnit.AMPS))
    fun getAvgCurrent(): Double = getCurrent().average()
}
