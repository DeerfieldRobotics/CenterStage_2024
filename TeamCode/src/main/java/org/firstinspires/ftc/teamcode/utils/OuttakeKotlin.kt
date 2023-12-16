package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.ServoImplEx

class OuttakeKotlin (hardwareMap: HardwareMap, private var slide: SlideKotlin) {

    private val armServo: ServoImplEx = hardwareMap.get("as") as ServoImplEx //control hub: 0
    private val wristServo: ServoImplEx = hardwareMap.get("ws") as ServoImplEx //control hub: 1
    private val gateServo: ServoImplEx = hardwareMap.get("gs") as ServoImplEx //control hub: 2

    private val armStartAngle = 42.5 //angle of arm at position 0.0 relative to horizontal, positive values ccw, towards outside of robot
    private val armEndAngle = -174.0 //angle of arm at position 1.0
    private val armInAngle = -132.0 //angle of arm for intaking
    private var armOutAngle = -30.8969 //angle of arm when it is out of the robot
    private val armDownAngle = -117.0 //angle of arm to clear low u channel
    private var currentArmAngle = armInAngle //current arm angle
    private val incrementMultiplier = -2.0 //multiplier for how much the arm angle changes when the outtake angle is adjusted

    private val wristStartAngle = -164.5 //angle of wrist at position 0.0 relative to the arm, positive values flips claw upwards
    private val wristEndAngle = 81.0 //angle of wrist at position 1.0
    private val wristInAngle = 65.5 //angle of wrist for intaking
    private val wristOutAngle = 8.48 //angle of wrist when it is out of the robot
    private val wristDownAngle = 30.0 //angle of wrist to clear low u channel
    private var currentWristAngle = wristInAngle //current wrist angle

    private val gateOuttake = 0.8 //open position
    private val gateIntake = 0.7 //intake position
    private val gateClose = 0.92 //closed position

    var outtakeExtended = false //whether the outtake is out or in
    var intakePosition = false //whether the outtake is in the intake position
    var gateClosed = true //whether the gate is open or closed

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
    fun outtakeAngleAdjust(armAngleIncrement: Double) {
        if(outtakeExtended) {
            armOutAngle += armAngleIncrement*incrementMultiplier
            setOuttakeAngle(currentArmAngle, wristOutAngle, true)
        }
    }
    fun update() {
        gateServo.position = when {
            outtakeExtended && gateClosed -> gateClose
            outtakeExtended -> gateOuttake
            gateClosed -> gateClose
            else -> gateIntake
        }
        setOuttakeAngle(when {
            outtakeExtended -> armOutAngle
            intakePosition -> armInAngle
            else -> armDownAngle
        }, when {
            outtakeExtended -> wristOutAngle
            intakePosition -> wristInAngle
            else -> wristDownAngle
        }, intakePosition||outtakeExtended)
    }
    fun outtakeProcedure(toggle:Boolean) {
        if(toggle && !outtakeExtended) { //makes sure outtake is not already out or currently going out
            t?.interrupt() //stops any existing threads
            t = Thread { //makes a new thread to run the outtake procedure
                gateClosed = true //close gate
                intakePosition = false
                while(true) {
                    if (slide.getPosition()
                            .average() <= slide.minSlideHeight
                    ) {
                        outtakeExtended = true
                        t?.interrupt() //stops any existing threads
                        break
                    }
                }
                t?.interrupt() //stops any existing threads
            }
            t!!.start()
        }
        else if (!toggle && outtakeExtended) { //makes sure outtake is not already in or currently going in
            t?.interrupt() //stops any existing threads
            t = Thread { //makes a new thread to run the outtake procedure
                gateClosed = true //open gate
                var currentTime = 0L
                while(true) {
                    if (slide.getPosition()
                            .average() <= slide.minSlideHeight
                    ) {
                        outtakeExtended = false //bring outtake in
                        currentTime = System.currentTimeMillis()
                        break
                    }
                }
                while(true) {
                    if(slide.getPosition().average() <= slide.minSlideHeight && System.currentTimeMillis() - currentTime > 500){
                        slide.bottomOutProcedure() //bottom out slide
                        break
                    }
                }
                intakePosition = false
                t?.interrupt() //stops any existing threads
            }
            t!!.start()
        }
    }
    fun outtakeProcedure() {
        outtakeProcedure(!outtakeExtended)
    }
    fun getOuttake():Boolean = outtakeExtended

    init {
        setOuttakeAngle(armDownAngle, wristDownAngle, false)
        gateServo.position = gateClose
    }
}