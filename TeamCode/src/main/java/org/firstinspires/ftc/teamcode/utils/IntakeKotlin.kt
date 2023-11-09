package org.firstinspires.ftc.teamcode.utils

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.ServoImplEx

class IntakeKotlin (hardwareMap: HardwareMap, private var slide: SlideKotlin){
    private var intakeServo: ServoImplEx = hardwareMap.get("is") as ServoImplEx //control hub: 5
    public var outtakeServo: ServoImplEx = hardwareMap.get("os") as ServoImplEx //control hub: 0 //TODO change to private if we change the Red1Right.java code
    private var armServo: ServoImplEx = hardwareMap.get("as") as ServoImplEx //control hub: 1
    private var intakeMotor: DcMotorEx = hardwareMap.get("im") as DcMotorEx  //expansion hub: 2

    private var intakeStart: Double = 0.4
    private var intakePositions: Array<Double> = arrayOf(0.0,0.10,0.20,0.3,0.4) //array of positions for the intake servo to go to

    private var outtakeClosed: Double = 0.0 //closed position
    private var outtakeOpen: Double = 0.34 //open position
    private var outtake: Boolean = true

    private var armOut: Double = 0.22
    private var armIn: Double = 0.46
    private var arm: Boolean = false

    private var timeSinceArm: Long = 0
    private var timeSinceOuttake: Long = 0
    private var minArmTimeIn = 200 //was 700
    private var minOuttakeTime = 100

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
    /*
    * Intake Procedure:
    * toggle: true to toggle intake in, and false to toggle intake out
     */
    fun intakeProcedure (toggle: Boolean) {
        if (toggle) { //brings intake in and down
            outtakeToggle(true) //ensure gate is open
            armToggle(false) //bring arm in
            if (System.currentTimeMillis() - timeSinceArm > minArmTimeIn) { //make sure arm is in before sliding down
                slide.bottomOut() //slide down and reset encoders
            } else {
                if (slide.getTargetPosition()[0] != slide.minSlideHeight) {
                    slide.setTargetPosition(slide.minSlideHeight) //if arm not in, then just chill
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION)
                    slide.setPower(-1.0)
                }
            }
        } else { //brings intake up and out
            outtakeToggle(false) //close gate
            if (slide.getPosition().average() > slide.minSlideHeight &&
                (slide.getTargetPosition()[0] != slide.minSlideHeight || slide.getMode()[0] != DcMotor.RunMode.RUN_TO_POSITION || slide.getPower()[0]!=1.0) && System.currentTimeMillis() - timeSinceOuttake > minOuttakeTime) { //wait for outtake to close and to get to right height
                slide.setTargetPosition(slide.minSlideHeight) //if we chilling then go to right slide height
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION)
                slide.setPower(1.0)
            }
            if (slide.getPosition().average() <= slide.minSlideHeight+50) { //if above minimum height and outtake has closed, then arm out
                armToggle(true) //once we clear the minimum height we bring that schlong out
            }
        }
    }



    fun intakeProcedure (toggle: Boolean, target: Int) { //this shit will be threaded fr
        val t = Thread {
            if (toggle) { //brings intake in and down
                outtakeToggle(true) //ensure gate is open
                armToggle(false) //bring arm in
                if (System.currentTimeMillis() - timeSinceArm > minArmTimeIn) { //make sure arm is in before sliding down
                    slide.bottomOut() //slide down and reset encoders
                } else {
                    if (slide.getTargetPosition()[0] != slide.minSlideHeight) {
                        slide.setTargetPosition(slide.minSlideHeight) //if arm not in, then just chill
                        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION)
                        slide.setPower(-1.0)
                    }
                }
            } else { //brings intake up and out
                outtakeToggle(false) //close gate
                if (slide.getPosition().average() >= target && slide.getTargetPosition()[0] != target && System.currentTimeMillis() - timeSinceOuttake > minOuttakeTime) { //wait for outtake to close and to get to right height
                    slide.setTargetPosition(target) //if we chilling then go to right slide height
                    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION)
                    slide.setPower(1.0)
                }
                if (slide.getPosition().average() <= slide.minSlideHeight+50) { //if above minimum height and outtake has closed, then arm out
                    armToggle(true) //once we clear the minimum height we bring that schlong out
                }
            }
        }
        t.start()
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