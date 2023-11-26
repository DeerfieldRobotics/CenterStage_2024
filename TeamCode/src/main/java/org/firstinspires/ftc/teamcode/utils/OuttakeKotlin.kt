package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.ServoImplEx

class OuttakeKotli (hardwareMap: HardwareMap, slide: SlideKotlin) {
    private var armServo: ServoImplEx = hardwareMap.get("as") as ServoImplEx //control hub:
    private var wristServo: ServoImplEx = hardwareMap.get("ws") as ServoImplEx //control hub:
    private var leftClaw: ServoImplEx = hardwareMap.get("lc") as ServoImplEx //control hub:
    private var rightClaw: ServoImplEx = hardwareMap.get("rc") as ServoImplEx //control hub:

    private var armStartAngle = 84.0 //angle of arm at position 0.0 relative to horizontal, positive values ccw, towards outside of robot
    private var armEndAngle = -200.0 //angle of arm at position 1.0
    private var armInAngle = 999.0 //angle of arm when it is in the robot TODO
    private var armOutAngle = 999.0 //angle of arm when it is out of the robot TODO

    private var wristStartAngle = -219.0 //angle of wrist at position 0.0 relative to the arm, positive values flips claw upwards
    private var wristEndAngle = 81.0 //angle of wrist at position 1.0
    private var wristInAngle = 999.0 //angle of wrist when it is in the robot TODO
    private var wristOutAngle = -120.0 //angle of wrist when it is out of the robot

    private var leftClawOpen = 0.0 //open position TODO
    private var leftClawClosed = 0.5 //closed position TODO

    private var rightClawOpen = 0.0 //open position TODO
    private var rightClawClosed = 0.5 //closed position TODO

    fun setOuttakeAngle(armAngle: Double, wristAngle: Double, absPos: Boolean) { //set position of arm and wrist servos, absPos is if the angle is absolute or relative to the arm
        armServo.position = (armAngle - armStartAngle) / (armEndAngle - armStartAngle)
        wristServo.position = if(absPos) (wristAngle + armAngle - wristStartAngle) / (wristEndAngle - wristStartAngle) else (wristAngle - wristStartAngle) / (wristEndAngle - wristStartAngle)
    }

    fun leftClawToggle() {
        leftClaw.position = if (leftClaw.position == leftClawOpen) leftClawClosed else leftClawOpen
    }
    fun leftClawToggle(toggle: Boolean) {
        leftClaw.position = if (toggle) leftClawOpen else leftClawClosed
    }

    fun rightClawToggle() {
        rightClaw.position = if (rightClaw.position == rightClawOpen) rightClawClosed else rightClawOpen
    }
    fun rightClawToggle(toggle: Boolean) {
        rightClaw.position = if (toggle) rightClawOpen else rightClawClosed
    }

    fun outtakeProcedure() {
        leftClawToggle(false) //close both claws
        rightClawToggle(false)
        setOuttakeAngle(armOutAngle, wristOutAngle, true)
    }

    init {
        armServo.position = armInAngle
        wristServo.position = wristInAngle
        leftClaw.position = leftClawOpen
        rightClaw.position = rightClawOpen
    }
}