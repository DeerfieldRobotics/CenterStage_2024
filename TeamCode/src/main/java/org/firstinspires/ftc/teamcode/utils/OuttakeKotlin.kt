package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.ServoImplEx

class OuttakeKotlin (hardwareMap: HardwareMap, private var slide: SlideKotlin) {
    private val armServo: ServoImplEx = hardwareMap.get("as") as ServoImplEx //control hub: 0
    private val wristServo: ServoImplEx = hardwareMap.get("ws") as ServoImplEx //control hub: 1
    private val gateServo: ServoImplEx = hardwareMap.get("gs") as ServoImplEx //control hub: 2

    var outtakePosition = OuttakePositions.INSIDE

    private val armStartAngle = 42.5 //angle of arm at position 0.0 relative to horizontal, positive values ccw, towards outside of robot
    private val armEndAngle = -174.0 //angle of arm at position 1.0

    private val incrementMultiplier = -2.0 //multiplier for how much the arm angle changes when the outtake angle is adjusted

    private val wristStartAngle = -164.5 //angle of wrist at position 0.0 relative to the arm, positive values flips claw upwards
    private val wristEndAngle = 81.0 //angle of wrist at position 1.0

    private val gateOuttake = 0.8 //open position
    private val gateIntake = 0.7 //intake position
    private val gateClose = 0.92 //closed position

    var outtakeProcedureTarget = OuttakePositions.OUTSIDE //target position for outtake procedure

    var gateClosed = true //whether the gate is open or closed

    private val transferKinematics = OuttakeKinematics(-130.0, 60.0, true)
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
    private class OuttakeKinematics (var armAngle: Double, var wristAngle: Double, var absPos: Boolean)
    private var currentWristAngle = outtakePositionMap[outtakePosition]!!.wristAngle //current wrist angle
    private var currentArmAngle = outtakePositionMap[outtakePosition]!!.armAngle //current arm angle
    fun setOuttakeKinematics(armAngle: Double, wristAngle: Double, absPos: Boolean) { //set position of arm and wrist servos, absPos is if the angle is absolute or relative to the arm
        armServo.position = (armAngle - armStartAngle) / (armEndAngle - armStartAngle)
        currentArmAngle = armAngle
        wristServo.position = if(absPos)
                (wristAngle + armAngle - wristStartAngle) / (wristEndAngle - wristStartAngle)
        else
                (wristAngle - wristStartAngle) / (wristEndAngle - wristStartAngle)
        currentWristAngle = wristAngle
    }
    fun getOuttakeAngle(): DoubleArray { //get position of arm and wrist servos
        return doubleArrayOf(currentArmAngle, currentWristAngle)
    }
    fun outtakeAngleAdjust(armAngleIncrement: Double) {
        if(outtakePosition == OuttakePositions.OUTSIDE) {
            outsideKinematics.armAngle += armAngleIncrement*incrementMultiplier
            update()
        }
    }
    fun update() {
        if(outtakeProcedureTarget != outtakePosition) {
            when (outtakeProcedureTarget) {
                OuttakePositions.OUTSIDE -> {
                    gateClosed = true
                    outtakePosition = OuttakePositions.INSIDE
                    if(slide.getPosition().average() <= slide.minSlideHeight) {
                        outtakePosition = OuttakePositions.OUTSIDE
                    }
                }
                OuttakePositions.INSIDE -> {
                    gateClosed = false
                    var currentTime = 0L
                    if(slide.getPosition().average() <= slide.minSlideHeight) {
                        setOuttakeKinematics(insideKinematics.armAngle, insideKinematics.wristAngle, insideKinematics.absPos)
                        currentTime = System.currentTimeMillis()
                    }
                    if(System.currentTimeMillis() - currentTime > 500 && outtakePosition == OuttakePositions.INSIDE) {
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
                    if(System.currentTimeMillis() - currentTime > 500 && outtakePosition == OuttakePositions.INSIDE) {
                        slide.bottomOut()
                        if(slide.bottomOut) {
                            outtakePosition = OuttakePositions.TRANSFER
                        }
                    }
                }
            }
        }
        gateServo.position = when {
            outtakePosition == OuttakePositions.OUTSIDE && gateClosed -> gateClose
            outtakePosition == OuttakePositions.OUTSIDE -> gateOuttake
            gateClosed -> gateClose
            else -> gateIntake
        }
        setOuttakeKinematics(
            outtakePositionMap[outtakePosition]!!.armAngle,
            outtakePositionMap[outtakePosition]!!.wristAngle,
            outtakePositionMap[outtakePosition]!!.absPos
        )
    }
    init {
        update()
        gateServo.position = gateClose
    }
}
