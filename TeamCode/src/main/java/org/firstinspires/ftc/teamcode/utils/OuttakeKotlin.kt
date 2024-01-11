package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.util.Range
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.ServoImplEx

class OuttakeKotlin (hardwareMap: HardwareMap, private var slide: SlideKotlin) {
    private val armServo: ServoImplEx = hardwareMap.get("as") as ServoImplEx //control hub: 0
    private val wristServo: ServoImplEx = hardwareMap.get("ws") as ServoImplEx //control hub: 1
    private val gateServo: ServoImplEx = hardwareMap.get("gs") as ServoImplEx //control hub: 2

    private val armStartAngle = -160.0 //angle of arm at position 0.0 relative to horizontal, positive values ccw, towards outside of robot
    private val armEndAngle = 90.0 //angle of arm at position 1.0

    private val wristStartAngle = 0.0 //angle of wrist at position 0.0 relative to the arm, positive values flips claw upwards
    private val wristEndAngle = 180.0 //angle of wrist at position 1.0

    private val gateOuttake = 0.8 //open position
    private val gateIntake = 0.7 //intake position
    private val gateClose = 0.92 //closed position

    private val incrementMultiplier = -2.0 //multiplier for how much the arm angle changes when the outtake angle is adjusted

    var outtakePosition = OuttakePositions.INSIDE //Only use to get current position
        private set
    var outtakeProcedureTarget = OuttakePositions.INSIDE //target position for outtake procedure, use to change outtake position

    var gateClosed = true //whether the gate is open or closed

    //Position definitions for outtake
    private val transferKinematics = OuttakeKinematics(-130.0, 60.0, true) //TODO
    private val insideKinematics = OuttakeKinematics(-117.0, 30.0, false)
    private var outsideKinematics = OuttakeKinematics(-30.8969, 8.48, true)

    enum class OuttakePositions {
        TRANSFER, INSIDE, OUTSIDE //TRANSFER for transferring, INSIDE for inside robot, OUTSIDE for out of robot
    }
    
    private val outtakePositionMap = mapOf(
        OuttakePositions.TRANSFER to transferKinematics,
        OuttakePositions.INSIDE to insideKinematics,
        OuttakePositions.OUTSIDE to outsideKinematics
    )

    private class OuttakeKinematics (var armAngle: Double, var wristAngle: Double, var absPos: Boolean) //subclass for outtake kinematics behavior 

    private var currentWristAngle = outtakePositionMap[outtakePosition]!!.wristAngle //current wrist angle
    private var currentArmAngle = outtakePositionMap[outtakePosition]!!.armAngle //current arm angle

    init {
        wristServo.scaleRange(0.24,1.0)
        update()
        gateServo.position = gateClose
    }

    fun setOuttakeKinematics(armAngle: Double, wristAngle: Double, absPos: Boolean) { //set position of arm and wrist servos, absPos is if angle is absolute or relative to arm
        armServo.position = (armAngle - armStartAngle) / (armEndAngle - armStartAngle)
        currentArmAngle = Range.clip(armAngle, armStartAngle, armEndAngle)
        wristServo.position = if(absPos)
                (wristAngle + armAngle - wristStartAngle) / (wristEndAngle - wristStartAngle)
        else
                (wristAngle - wristStartAngle) / (wristEndAngle - wristStartAngle)
        currentWristAngle = Range.clip(wristAngle, wristStartAngle, wristEndAngle)
    }

    fun getOuttakeAngle() = arrayOf(currentArmAngle, currentWristAngle)

    fun outtakeAngleAdjust(armAngleIncrement: Double) {
        if(outtakePosition == OuttakePositions.OUTSIDE)
            outsideKinematics.armAngle += armAngleIncrement*incrementMultiplier
    }

    fun update() {
        outtakeProcedure()
        gateServo.position = when {
            gateClosed -> gateClose
            outtakePosition == OuttakePositions.OUTSIDE -> gateOuttake
            else -> gateIntake
        }
        setOuttakeKinematics(
            outtakePositionMap[outtakePosition]!!.armAngle,
            outtakePositionMap[outtakePosition]!!.wristAngle,
            outtakePositionMap[outtakePosition]!!.absPos
        )
    }

    private fun outtakeProcedure() {
        if(outtakeProcedureTarget != outtakePosition) {
            when (outtakeProcedureTarget) {
                OuttakePositions.OUTSIDE -> {
                    gateClosed = true
                    outtakePosition = OuttakePositions.INSIDE
                    if(slide.getPosition().average() <= slide.minSlideHeight) //When slide is high enough, bring arm out
                        outtakePosition = OuttakePositions.OUTSIDE
                }
                OuttakePositions.INSIDE -> {
                    gateClosed = true
                    if(outtakePosition == OuttakePositions.TRANSFER) //Guard clause-ish, if already transfer, just go to inside position
                        outtakePosition = OuttakePositions.INSIDE
                    var currentTime = 0L
                    if(slide.getPosition().average() <= slide.minSlideHeight) {
                        setOuttakeKinematics(insideKinematics.armAngle, insideKinematics.wristAngle, insideKinematics.absPos) //Need to use manual command in order to not break out of condition
                        currentTime = System.currentTimeMillis()
                    }
                    if(System.currentTimeMillis() - currentTime > 500 && currentTime != 0L) {
                        slide.bottomOut()
                        if(slide.bottomOut)
                            outtakePosition = OuttakePositions.INSIDE
                    }
                }
                OuttakePositions.TRANSFER -> {
                    gateClosed = false
                    var currentTime = 0L
                    if(slide.getPosition().average() <= slide.minSlideHeight) {
                        outtakePosition = OuttakePositions.INSIDE
                        currentTime = System.currentTimeMillis()
                    }
                    if(System.currentTimeMillis() - currentTime > 500 && outtakePosition == OuttakePositions.INSIDE) { //if its already inside, current time will be 0, thus it will be greater than 500
                        slide.bottomOut()
                        if(slide.bottomOut)
                            outtakePosition = OuttakePositions.TRANSFER
                    }
                }
            }
        }
    }

}
