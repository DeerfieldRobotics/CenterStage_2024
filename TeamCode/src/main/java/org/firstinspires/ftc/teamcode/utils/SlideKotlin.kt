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
    var slideTolerance = 100
    var bottomOut = false
    var bottomOutProcedure = false

    private var slide1Position = 0
    private var slide2Position = 0

    private var overCurrent = false

    private var t: Thread? = null

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
    private fun bottomOut () {
        if(!bottomOut) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER)
            setPower(1.0)
        }
        if (overCurrent) {
            bottomOut = true
            setPower(0.0)
            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
            setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        }
    }
    fun bottomOutProcedure () {
        t?.interrupt()
        if(!bottomOut) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER)
            setPower(1.0)
            t = Thread {
                val currentTime = System.currentTimeMillis()
                while(!bottomOut && System.currentTimeMillis() - currentTime < 5000) {
                    bottomOutProcedure = true
                }
                t?.interrupt()
            }
            t!!.start()
        }
    }
    fun checkBottomOut () {
        if ((overCurrent) && (getPosition()[0] > -200 || getPosition()[1] > -200)) {
            bottomOut = true
            bottomOutProcedure = false
            setPower(0.0)
            setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
            setMode(DcMotor.RunMode.RUN_USING_ENCODER)
        }
    }
    fun update() {
        slide1Position = slide1.currentPosition
        slide2Position = slide2.currentPosition
        overCurrent = slide1.isOverCurrent || slide2.isOverCurrent
        if(bottomOutProcedure && !bottomOut) {
            bottomOut()
        }
        else if (bottomOutProcedure) {
            bottomOutProcedure = false
        }
        checkBottomOut()
    }

    fun getMotors(): Array<DcMotorEx> = arrayOf(slide1, slide2)
    fun getPosition(): Array<Int> = arrayOf(slide1Position, slide2Position)
    fun getTargetPosition(): Array<Int> = arrayOf(slide1.targetPosition, slide2.targetPosition)
    fun getCurrent(): Array<Double> = arrayOf(slide1.getCurrent(CurrentUnit.AMPS), slide2.getCurrent(CurrentUnit.AMPS))
    fun getAvgCurrent(): Double = getCurrent().average()
    fun getPower(): Array<Double> = arrayOf(slide1.power, slide2.power)
    fun getMode(): Array<DcMotor.RunMode> = arrayOf(slide1.mode, slide2.mode)
    fun isBusy(): Boolean = slide1.isBusy || slide2.isBusy
}