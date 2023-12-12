package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2

class OuttakeKotlin (hardwareMap: HardwareMap, private var slide: SlideKotlin) {

    private val armServo: ServoImplEx = hardwareMap.get("as") as ServoImplEx //control hub: 0
    private val wristServo: ServoImplEx = hardwareMap.get("ws") as ServoImplEx //control hub: 1
    private val gateServo: ServoImplEx = hardwareMap.get("gs") as ServoImplEx //control hub: 2

    private val armStartAngle = 42.5 //angle of arm at position 0.0 relative to horizontal, positive values ccw, towards outside of robot
    private val armEndAngle = -174.0 //angle of arm at position 1.0
    private val armInAngle = -132.0 //angle of arm when it is in the robot
    private val armOutAngle = -30.8969 //angle of arm when it is out of the robot
    private val armDownAngle = -120.0 //angle of arm to clear low u channel
    private var currentArmAngle = armInAngle //current arm angle
    private val incrementMultiplier = -2.0 //multiplier for how much the arm angle changes when the outtake angle is adjusted
    private var arm = false //whether the arm is in or out

    private val wristStartAngle = -164.5 //angle of wrist at position 0.0 relative to the arm, positive values flips claw upwards
    private val wristEndAngle = 81.0 //angle of wrist at position 1.0
    private val wristInAngle = 65.5 //angle of wrist when it is in the robot
    private val wristOutAngle = 8.48 //angle of wrist when it is out of the robot
    private val wristDownAngle = 82.0 //angle of wrist to clear low u channel
    private var currentWristAngle = wristInAngle //current wrist angle

    private val gateOut = 0.8 //open position TODO
    private val gateIntake = 0.7 //intake position TODO
    private val gateClosed = 0.92 //closed position TODO

    private var outtake = false //whether the outtake is out or in
    private var gate = false //whether the gate is open or closed

    private var t: Thread? = null

    fun setOuttakeAngle(armAngle: Double, wristAngle: Double, absPos: Boolean) { //set position of arm and wrist servos, absPos is if the angle is absolute or relative to the arm
        armServo.position = (armAngle - armStartAngle) / (armEndAngle - armStartAngle)
        currentArmAngle = armAngle
        wristServo.position = if(absPos) (wristAngle + armAngle - wristStartAngle) / (wristEndAngle - wristStartAngle) else (wristAngle - wristStartAngle) / (wristEndAngle - wristStartAngle)
        currentWristAngle = wristAngle
    }
    fun getOuttakeAngle(): DoubleArray { //get position of arm and wrist servos
        return doubleArrayOf(currentArmAngle, currentWristAngle)
    }

    fun gateToggle() {
        gateToggle(!gate)
    }
    fun gateToggle(toggle: Boolean) {
        if(gate != toggle)
            if(outtake)
                if(gate) {
                    gateServo.position = gateOut
                    gate = false
                }
                else {
                    gateServo.position = gateClosed
                    gate = true
                }
            else
                if(gate) {
                    gateServo.position = gateIntake
                    gate = false
                }
                else {
                    gateServo.position = gateClosed
                    gate = true
                }
    }
    fun armToggle() {
        armToggle(!arm)
    }
    fun armToggle(toggle:Boolean) {
        if(toggle!=arm)
            if (toggle) {
                arm = true
                outtake = true
                setOuttakeAngle(armOutAngle, wristOutAngle, true) //bring arm out and wrist down to correct angle
            } else {
                arm = false
                outtake = false
                setOuttakeAngle(armDownAngle, wristDownAngle, true) //bring arm out and wrist down to correct angle
            }
    }
    fun intakePosition() {
        setOuttakeAngle(armInAngle, wristInAngle, true)
    }
    fun outtakeAngleAdjust(armAngleIncrement: Double) {
        if(outtake) {
            currentArmAngle += armAngleIncrement*incrementMultiplier
            setOuttakeAngle(currentArmAngle, wristOutAngle, true)
        }
    }
    fun outtakeProcedure(toggle:Boolean) {
        if(toggle && !outtake) { //makes sure outtake is not already out or currently going out
            t?.interrupt() //stops any existing threads
            t = Thread { //makes a new thread to run the outtake procedure
                gateToggle(true) //close gate
                armToggle(false)
                while(true) {
                    if (slide.getPosition()
                            .average() <= slide.minSlideHeight
                    ) {
                        outtake = true
                        arm = true
                        armToggle(true) //bring arm out and wrist down to correct angle
                        break
                    }
                }
                t?.interrupt() //stops any existing threads
            }
            t!!.start()
        }
        else if (!toggle && outtake) { //makes sure outtake is not already in or currently going in
            outtake = false
            t?.interrupt() //stops any existing threads
            t = Thread { //makes a new thread to run the outtake procedure
                gateToggle(false) //open gate
                var currentTime = 0L
                while(true) {
                    if (slide.getPosition()
                            .average() <= slide.minSlideHeight
                    ) {
                        armToggle(false) //bring arm in and wrist up to correct angle
                        currentTime = System.currentTimeMillis()
                        break
                    }
                }
                while(true) {
                    if(slide.getPosition().average() <= slide.minSlideHeight && System.currentTimeMillis() - currentTime > 500){
                        slide.bottomOutProcedure() //bottom out slide
                        break
                    }
                    else {
                        slide.setPower(0.0)
                    }
                }
                intakePosition()
                t?.interrupt() //stops any existing threads
            }
            t!!.start()
        }
    }
    fun outtakeProcedure() {
        outtakeProcedure(!outtake)
    }
    fun getOuttake():Boolean = outtake

    init {
        setOuttakeAngle(armInAngle, wristInAngle, true)
        gateServo.position = gateOut
    }
}