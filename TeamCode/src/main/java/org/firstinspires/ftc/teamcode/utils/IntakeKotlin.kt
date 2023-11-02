package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.ServoImplEx

class IntakeKotlin (hardwareMap: HardwareMap){


    private var intakeServo: ServoImplEx = hardwareMap.get("is") as ServoImplEx //control hub: 5
    private var outtakeServo: ServoImplEx = hardwareMap.get("os") as ServoImplEx //control hub: 0
    private var armServo: ServoImplEx = hardwareMap.get("as") as ServoImplEx //control hub: 1
    private var intakeMotor: DcMotorEx = hardwareMap.get("im") as DcMotorEx  //expansion hub: 2

    private var crossPressed = false
    private var trianglePressed = false

    private var intakeStart: Double = 0.4
    private var intakeEnd: Double = 0.0
    private var intakePositions: Array<Double> = arrayOf(0.0,0.15,0.25,0.3,0.35, .4); //array of positions for the intake servo to go to

    private var outtakeClosed: Double = 0.0 //closed position
    private var outtakeOpen: Double = 0.34 //open position
    private var outtake: Boolean = true

    private var armOut: Double = 0.05
    private var armIn: Double = 0.46
    private var arm: Boolean = false

    var timeSinceArm: Long = 0
    var timeSinceOuttake: Long = 0

    init {
        intakeServo.position = intakeStart
        outtakeServo.position = outtakeClosed
        armServo.position = armIn
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        intakeMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }
    fun intakeServo(position: Int) {
        intakeServo.position = intakePositions[position]
    }
    fun outtakeToggle (){ //toggles between open and closed positions
        if (outtake) {
            outtake = false
            outtakeServo.position = outtakeClosed
        } else {
            outtake = true
            outtakeServo.position = outtakeOpen
        }
    }

    fun outtakeToggle(toggle:Boolean) {
        if(toggle!=outtake) {
            timeSinceOuttake = System.currentTimeMillis()
            if (toggle) {
                outtake = true
                outtakeServo.position = outtakeOpen
            } else {
                outtake = false
                outtakeServo.position = outtakeClosed
            }
        }
    }
    fun armToggle() {
        if(arm) {
            arm = false
            armServo.position = armIn
        } else {
            arm = true
            armServo.position = armOut
        }
    }
    fun armToggle(toggle:Boolean) {
        if(toggle!=arm) {
            timeSinceArm = System.currentTimeMillis()
            if (toggle) {
                arm = true
                armServo.position = armOut
            } else {
                arm = false
                armServo.position = armIn
            }
        }
    }

    fun intakeServo (position: Double){
        intakeServo.position = position
    }
    fun armServo(position:Double){
        armServo.position = position
    }
    fun intake (power: Double) {
        intakeMotor.power = power
    }
    fun getIntakePos():Double = intakeServo.position
    fun getOuttakePos():Double = outtakeServo.position
    fun getArmPos():Double = armServo.position
}