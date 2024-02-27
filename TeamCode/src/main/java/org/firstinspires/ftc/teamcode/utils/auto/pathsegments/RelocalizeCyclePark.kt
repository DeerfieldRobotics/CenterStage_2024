package org.firstinspires.ftc.teamcode.utils.auto.pathsegments

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper
import org.firstinspires.ftc.teamcode.utils.detection.AllianceHelper
import org.firstinspires.ftc.teamcode.utils.hardware.Intake

class RelocalizeCyclePark(
    override val robot: Robot, private val cycle: Int
) : RoadrunnerPathSegment(robot) {
    override lateinit var trajectorySequenceBuilder: TrajectorySequenceBuilder

    override fun buildPathSegment() {
        trajectorySequenceBuilder = robot.drive.trajectorySequenceBuilder(PoseHelper.currentPose)
            .addTemporalMarker {
//                robot.backCameraPortal?.close() BROKEN IN 9.1 TODO
                robot.backCameraPortal?.stopStreaming() //WORKAROUND
            }
            .addTemporalMarker {
                if (PoseHelper.path == PoseHelper.Path.OUTSIDE)
                    robot.intake.servoPosition = Intake.IntakePositions.THREE
                else if(cycle == 0)
                    robot.intake.servoPosition = Intake.IntakePositions.FOUR
                else
                    robot.intake.servoPosition = Intake.IntakePositions.TWO
                robot.intake.update();
            }
            .setTangent(Math.toRadians(if (PoseHelper.path == PoseHelper.Path.INSIDE) 140.0 else -145.0) * PoseHelper.allianceAngleMultiplier)
            .splineToConstantHeading(PoseHelper.wingTruss.vec(), Math.toRadians(180.0))
            .addTemporalMarker(::intake)
            .splineToConstantHeading(
                PoseHelper.stackPose.plus(PoseHelper.stackOffset).vec(),
                Math.toRadians(if (PoseHelper.path == PoseHelper.Path.INSIDE) 180.0 else 120.0 * PoseHelper.allianceAngleMultiplier)
            )
            //TO PARK
            .setVelConstraint(PoseHelper.blastVelocityConstraint)
            .setAccelConstraint(PoseHelper.blastAccelerationConstraint)
            .addTemporalMarker(this::outtakeTransfer)
            .forward(1.5)
            .back(3.0)
            .addTemporalMarker(this::stopIntake)
            .addTemporalMarker(this::transfer)
            .setTangent(Math.toRadians(0.0))
            .splineToLinearHeading(PoseHelper.wingTruss, Math.toRadians(0.0))
            .splineToLinearHeading(PoseHelper.boardTruss, Math.toRadians(0.0))
            .splineToLinearHeading(PoseHelper.parkPose, Math.toRadians(0.0))
    }

    override fun toString() = "RelocalizeToBackboardPathSegment"
}