package org.firstinspires.ftc.teamcode.utils.auto.pathsegments

import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper
import org.firstinspires.ftc.teamcode.utils.hardware.Outtake

abstract class RoadrunnerPathSegment(open val robot: Robot): PathSegment{
    abstract var trajectorySequenceBuilder: TrajectorySequenceBuilder
    override var running = false
    override var duration = 0.0 //TODO

    abstract fun buildPathSegment()

    private fun getTrajectorySequence(): TrajectorySequence? {
        buildPathSegment()
        trajectorySequenceBuilder = trajectorySequenceBuilder.addTemporalMarker {
            PoseHelper.currentPose = robot.drive.poseEstimate
            running = false
        }
        return trajectorySequenceBuilder.build()
    }

    override fun followPathSegment() {
        running = true
        robot.drive.followTrajectorySequenceAsync(getTrajectorySequence())
    }

    //INTAKE METHODS
     fun intake() {
        robot.intake.intakePower(1.0)
        robot.intake.update()
    }

     fun stopIntake() {
        robot.intake.intakePower(0.0)
        robot.intake.update()
    }

     fun transfer() {
        robot.intake.transfer()
        robot.intake.update()
    }

     fun outtakePurple() {
        robot.intake.motorMode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        robot.intake.update()
        robot.intake.motorTargetPosition = -450
        robot.intake.update()
        robot.intake.motorMode = DcMotor.RunMode.RUN_TO_POSITION
        robot.intake.update()
        robot.intake.motorPower = 0.35
        robot.intake.update()
    }

    //OUTTAKE METHODS
     fun outtake() {
        robot.outtake.outtakeProcedureTarget = Outtake.OuttakePositions.OUTSIDE
        if (robot.slide.getAvgPosition() >= -900) setSlideHeight(-900)
    }

     fun outtakeIn() {
        if (robot.outtake.outtakePosition == Outtake.OuttakePositions.OUTSIDE) setSlideHeight(-1000)
        robot.outtake.outtakeProcedureTarget = Outtake.OuttakePositions.INSIDE
    }

     fun outtakeTransfer() {
        if (robot.outtake.outtakePosition == Outtake.OuttakePositions.OUTSIDE) setSlideHeight(-1000)
        robot.outtake.outtakeProcedureTarget = Outtake.OuttakePositions.TRANSFER
        robot.outtake.update()
    }

     fun drop() {
        robot.outtake.gateClosed = false
    }

    fun setSlideHeight(height: Int) {
        robot.slide.setTargetPosition(height)
        robot.slide.setMode(DcMotor.RunMode.RUN_TO_POSITION)
        robot.slide.power = 1.0
    }
}