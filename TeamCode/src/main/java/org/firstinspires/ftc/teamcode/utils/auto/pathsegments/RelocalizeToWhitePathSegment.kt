package org.firstinspires.ftc.teamcode.utils.auto.pathsegments

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper
import org.firstinspires.ftc.teamcode.utils.hardware.Intake

class RelocalizeToWhitePathSegment(
    override val robot: Robot
) : RoadrunnerPathSegment(robot) {
    override var trajectorySequenceBuilder: TrajectorySequenceBuilder =
        robot.drive.trajectorySequenceBuilder(
            PoseHelper.currentPose
        )

    override fun buildPathSegment() {
        trajectorySequenceBuilder = trajectorySequenceBuilder
            .addTemporalMarker {
                if (PoseHelper.path == PoseHelper.Path.OUTSIDE)
                    robot.intake.servoPosition = Intake.IntakePositions.THREE
                else {
                    robot.intake.servoPosition = Intake.IntakePositions.TWO
                }
                robot.intake.update();
            }
            .setTangent(Math.toRadians(if (PoseHelper.path == PoseHelper.Path.INSIDE) 140.0 else -145.0) * PoseHelper.allianceAngleMultiplier)
            .splineToConstantHeading(PoseHelper.wingTruss.vec(), Math.toRadians(180.0))
            .addTemporalMarker(::intake)
            .splineToConstantHeading(
                PoseHelper.stackPose.plus(PoseHelper.stackOffset).vec(),
                Math.toRadians(if (PoseHelper.path == PoseHelper.Path.INSIDE) 180.0 else 120.0 * PoseHelper.allianceAngleMultiplier)
            )
    }
}