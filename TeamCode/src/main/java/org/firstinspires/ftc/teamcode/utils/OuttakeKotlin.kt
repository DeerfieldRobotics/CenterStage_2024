package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.ServoImplEx

class OuttakeKotlin (hardwareMap: HardwareMap, private var slide: SlideKotlin) {

    private var armServo: ServoImplEx = hardwareMap.get("as") as ServoImplEx //control hub:
    private var wristServo: ServoImplEx = hardwareMap.get("ws") as ServoImplEx //control hub:
    private var leftClaw: ServoImplEx = hardwareMap.get("lc") as ServoImplEx //control hub:
    private var rightClaw: ServoImplEx = hardwareMap.get("rc") as ServoImplEx //control hub:

    private var armStartAngle = 84.0 //angle of arm at position 0.0 relative to horizontal, positive values ccw, towards outside of robot
    private var armEndAngle = -200.0 //angle of arm at position 1.0
    private var armInAngle = 999.0 //angle of arm when it is in the robot TODO
    private var armOutAngle = 999.0 //angle of arm when it is out of the robot TODO
    private var currentArmAngle = armInAngle //current arm angle
    private var incrementMultiplier = 1.0 //multiplier for how much the arm angle changes when the outtake angle is adjusted
    private var arm = false //whether the arm is in or out

    private var wristStartAngle = -219.0 //angle of wrist at position 0.0 relative to the arm, positive values flips claw upwards
    private var wristEndAngle = 81.0 //angle of wrist at position 1.0
    private var wristInAngle = 999.0 //angle of wrist when it is in the robot TODO
    private var wristOutAngle = -120.0 //angle of wrist when it is out of the robot

    private var leftClawOpen = 0.0 //open position TODO
    private var leftClawClosed = 0.5 //closed position TODO

    private var rightClawOpen = 0.0 //open position TODO
    private var rightClawClosed = 0.5 //closed position TODO

    private var outtake = false;

    private var t: Thread? = null

    private fun setOuttakeAngle(armAngle: Double, wristAngle: Double, absPos: Boolean) { //set position of arm and wrist servos, absPos is if the angle is absolute or relative to the arm
        armServo.position = (armAngle - armStartAngle) / (armEndAngle - armStartAngle)
        currentArmAngle = armAngle
        wristServo.position = if(absPos) (wristAngle + armAngle - wristStartAngle) / (wristEndAngle - wristStartAngle) else (wristAngle - wristStartAngle) / (wristEndAngle - wristStartAngle)
    }

    fun leftClawToggle() {
        leftClaw.position = if (leftClaw.position == leftClawOpen) leftClawClosed else leftClawOpen
    }
    fun leftClawToggle(toggle: Boolean) {
        leftClaw.position = if (toggle) leftClawClosed else leftClawOpen
    }

    fun rightClawToggle() {
        rightClaw.position = if (rightClaw.position == rightClawOpen) rightClawClosed else rightClawOpen
    }
    fun rightClawToggle(toggle: Boolean) {
        rightClaw.position = if (toggle) rightClawClosed else rightClawOpen
    }
    fun armToggle() {
        if (arm) { //if arm out, bring in
            arm = false
            setOuttakeAngle(armInAngle, wristInAngle, true) //bring arm out and wrist down to correct angle
        } else { //if arm in, bring out
            arm = true
            setOuttakeAngle(armOutAngle, wristOutAngle, true) //bring arm out and wrist down to correct angle
        }
    }
    fun armToggle(toggle:Boolean) {
        if(toggle!=arm) {
            if (toggle) {
                arm = true
                setOuttakeAngle(armOutAngle, wristOutAngle, true) //bring arm out and wrist down to correct angle
            } else {
                arm = false
                setOuttakeAngle(armInAngle, wristInAngle, true) //bring arm out and wrist down to correct angle
            }
        }
    }
    fun outtakeAngleAdjust(armAngleIncrement: Double) {
        if(outtake) {
            currentArmAngle += armAngleIncrement*incrementMultiplier
            setOuttakeAngle(currentArmAngle, wristOutAngle, true)
        }
    }
    fun outtakeProcedure(toggle:Boolean) {
        if(toggle && !outtake) { //makes sure outtake is not already out or currently going out
            outtake = true
            t?.interrupt() //stops any existing threads
            t = Thread { //makes a new thread to run the outtake procedure
                leftClawToggle(true) //close both claws
                rightClawToggle(true)
                while(true) {
                    if (slide.getPosition()
                            .average() <= slide.minSlideHeight
                    ) {
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
                leftClawToggle(false) //open both claws
                rightClawToggle(false)
                while(true) {
                    if (slide.getPosition()
                            .average() <= slide.minSlideHeight
                    ) {
                        armToggle(false) //bring arm in and wrist up to correct angle
                        break
                    }
                }
                t?.interrupt() //stops any existing threads
            }
            t!!.start()
        }
    }
    fun outtakeProcedure() {
        if(!outtake) { //makes sure outtake is not already out or currently going out
            outtakeProcedure(true)
        }
        else { //makes sure outtake is not already in or currently going in
            outtakeProcedure(false)
        }
    }

    init {
        armServo.position = armInAngle
        wristServo.position = wristInAngle
        leftClaw.position = leftClawOpen
        rightClaw.position = rightClawOpen
    }
}