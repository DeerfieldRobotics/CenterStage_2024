package org.firstinspires.ftc.teamcode.utils.hardware

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.ServoImplEx

class Outtake (hardwareMap: HardwareMap, private var slide: Slide) {
    private val armServo: ServoImplEx = hardwareMap.get("as") as ServoImplEx //control hub: 0
    private val wristServo: ServoImplEx = hardwareMap.get("ws") as ServoImplEx //control hub: 1
    private val gateServo: ServoImplEx = hardwareMap.get("gs") as ServoImplEx //control hub: 2

    private val armStartAngle = -160.0 //angle of arm at position 0.0 relative to horizontal, positive values ccw, towards outside of robot
    private val armEndAngle = 90.0 //angle of arm at position 1.0

    private val wristStartAngle = 0.0 //angle of wrist at position 0.0 relative to the arm, positive values flips claw upwards
    private val wristEndAngle = 180.0 //angle of wrist at position 1.0

    private val gateOpen = 0.52 //open position
    private val gateClose = 0.31 //closed position

    private var currentTime = 0L //current time for outtake procedure
    private var outtakeProcedureComplete = true
    private var currentTimeSet = true

    private val incrementMultiplier = -2.0 //multiplier for how much the arm angle changes when the outtake angle is adjusted

    var outtakePosition = OuttakePositions.INSIDE //Only use to get current position
        private set
    var outtakeProcedureTarget =
        OuttakePositions.INSIDE //target position for outtake procedure, use to change outtake position

    var gateClosed = true //whether the gate is open or closed

    //Position definitions for outtake
    private val transferKinematics = OuttakeKinematics(-102.5, 190.5, true)
    private val insideKinematics = OuttakeKinematics(-108.0, 180.0, false)
    private var outsideKinematics = OuttakeKinematics(-30.8969, 104.0, true)
    private var middleKinematics = OuttakeKinematics(-102.5, 112.0, false)

    enum class OuttakePositions {
        TRANSFER, INSIDE, OUTSIDE, MIDDLE //TRANSFER for transferring, INSIDE for inside robot, OUTSIDE for out of robot
    }
    
    private val outtakePositionMap = mapOf(
        OuttakePositions.TRANSFER to transferKinematics,
        OuttakePositions.INSIDE to insideKinematics,
        OuttakePositions.OUTSIDE to outsideKinematics,
        OuttakePositions.MIDDLE to middleKinematics
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
        wristServo.position = if(absPos)
                (wristAngle + armAngle - wristStartAngle) / (wristEndAngle - wristStartAngle)
        else
                (wristAngle - wristStartAngle) / (wristEndAngle - wristStartAngle)
    }

    fun getOuttakeAngle() = arrayOf(currentArmAngle, currentWristAngle)

    fun outtakeAngleAdjust(armAngleIncrement: Double) {
        if(outtakePosition == OuttakePositions.OUTSIDE)
            outsideKinematics.armAngle += armAngleIncrement*incrementMultiplier
    }

    fun update() {
        outtakeProcedure()
        gateServo.position = if(gateClosed) gateClose else gateOpen
        setOuttakeKinematics(
            outtakePositionMap[outtakePosition]!!.armAngle,
            outtakePositionMap[outtakePosition]!!.wristAngle,
            outtakePositionMap[outtakePosition]!!.absPos
        )
    }

    private fun outtakeProcedure() {
        if(outtakeProcedureTarget != outtakePosition || !outtakeProcedureComplete) {
            when (outtakeProcedureTarget) {
                OuttakePositions.OUTSIDE -> {
                    gateClosed = true
                    outtakePosition = OuttakePositions.INSIDE
                    if(slide.getPosition().average() <= slide.minSlideHeight) { //When slide is high enough, bring arm out
                        outtakePosition = OuttakePositions.OUTSIDE
                        outtakeProcedureComplete = true
                        currentTime = 0L
                    }
                }
                OuttakePositions.INSIDE -> {
                    gateClosed = true
                    if(outtakePosition == OuttakePositions.TRANSFER) //Guard clause-ish, if already transfer, just go to inside position
                        outtakePosition = OuttakePositions.INSIDE
                    if(slide.getPosition().average() <= slide.minSlideHeight && currentTime == 0L) { //When slide is high enough, bring arm in
                        outtakePosition = OuttakePositions.INSIDE
                        outtakeProcedureComplete = false
                        currentTime = System.currentTimeMillis()
                    }
                    else if(slide.getPosition().average() > slide.minSlideHeight && outtakePosition == OuttakePositions.OUTSIDE) { //If lower than slide height, reset current time
                        currentTime = 0L
                    }
                    if(System.currentTimeMillis() - currentTime > 500 && currentTime != 0L) {
                        currentTime = 0L
                        if(slide.bottomOut) {
                            outtakePosition = OuttakePositions.INSIDE
                            outtakeProcedureComplete = true
                        }
                        slide.bottomOutProcedure = true
                    }
                }
                OuttakePositions.TRANSFER -> {
                    gateClosed = false
                    if(slide.getPosition().average() <= slide.minSlideHeight && currentTime == 0L) { //When slide is high enough, bring arm in
                        outtakePosition = OuttakePositions.INSIDE
                        currentTime = System.currentTimeMillis()
                    }
                    else if(slide.getPosition().average() >= slide.minSlideHeight && outtakePosition == OuttakePositions.OUTSIDE) {
                        currentTime = 0L
                    }

                    if(System.currentTimeMillis() - currentTime > 500 && outtakePosition == OuttakePositions.INSIDE) { //if its already inside, current time will be 0, thus it will be greater than 500
                        if(slide.bottomOut) {

                            if(currentTime == 0L && !currentTimeSet) {
                                currentTime = System.currentTimeMillis()
                                currentTimeSet = true
                            }
                            else if(!currentTimeSet){
                                currentTime = 0L
                            }
                            outtakePosition = if(currentTime != 0L && System.currentTimeMillis() - currentTime < 300) {
                                OuttakePositions.MIDDLE
                            } else
                                OuttakePositions.TRANSFER
                        }
                        slide.bottomOutProcedure = true
                    }

                }
                OuttakePositions.MIDDLE -> {
                    gateClosed = false
                }
            }
        }
    }
}