package org.firstinspires.ftc.teamcode.utils.auto.pathsegments

import android.util.Log
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.roadrunner.drive.CogchampDrive
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.utils.Other.LogcatHelper
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper
import org.firstinspires.ftc.teamcode.utils.detection.AprilTagAlignmentProcessor
import org.firstinspires.ftc.teamcode.utils.hardware.Intake
import org.firstinspires.ftc.teamcode.utils.hardware.Outtake
import org.firstinspires.ftc.teamcode.utils.hardware.Slide

abstract class RoadrunnerPathSegment(open val drive: CogchampDrive, open val intake: Intake, open val slide: Slide, open val outtake: Outtake): PathSegment{
    abstract var trajectorySequenceBuilder: TrajectorySequenceBuilder
    override var running = false

    abstract fun buildPathSegment()

    private fun getTrajectorySequence(): TrajectorySequence? {
        buildPathSegment()
        trajectorySequenceBuilder = trajectorySequenceBuilder.addTemporalMarker {
            running = false
        }
        return trajectorySequenceBuilder.build()
    }

    override fun followPathSegment() {
        running = true
        drive.followTrajectorySequenceAsync(getTrajectorySequence())
    }

    //INTAKE METHODS
     fun intake() {
        intake.intakePower(1.0)
        intake.update()
    }

     fun stopIntake() {
        intake.intakePower(0.0)
        intake.update()
    }

     fun transfer() {
        intake.transfer()
        intake.update()
    }

     fun outtakePurple() {
        intake.motorMode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        intake.update()
        intake.motorTargetPosition = -450
        intake.update()
        intake.motorMode = DcMotor.RunMode.RUN_TO_POSITION
        intake.update()
        intake.motorPower = 0.35
        intake.update()
    }

    //OUTTAKE METHODS
     fun outtake() {
        outtake.outtakeProcedureTarget = Outtake.OuttakePositions.OUTSIDE
        if (slide.getAvgPosition() >= -900) setSlideHeight(-900)
    }

     fun outtakeIn() {
        if (outtake.outtakePosition == Outtake.OuttakePositions.OUTSIDE) setSlideHeight(-1000)
        outtake.outtakeProcedureTarget = Outtake.OuttakePositions.INSIDE
    }

     fun outtakeTransfer() {
        if (outtake.outtakePosition == Outtake.OuttakePositions.OUTSIDE) setSlideHeight(-1000)
        outtake.outtakeProcedureTarget = Outtake.OuttakePositions.TRANSFER
        outtake.update()
    }

     fun drop() {
        outtake.gateClosed = false
    }

    fun setSlideHeight(height: Int) {
        slide.setTargetPosition(height)
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION)
        slide.power = 1.0
    }
}