package org.firstinspires.ftc.teamcode.utils.auto.pathsegments

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper
import org.firstinspires.ftc.teamcode.utils.detection.AllianceHelper
import org.firstinspires.ftc.teamcode.utils.detection.ColorDetectionProcessor
import org.firstinspires.ftc.teamcode.utils.hardware.Intake

class CloseInitToBackboardPathSegment(
    override val robot: Robot
) : RoadrunnerPathSegment(robot) {

    override lateinit var trajectorySequenceBuilder: TrajectorySequenceBuilder

    override fun buildPathSegment() {
        trajectorySequenceBuilder = robot.drive.trajectorySequenceBuilder(PoseHelper.initPose)
//            .setVelConstraint(PoseHelper.toBackboardVelocityConstraint)
            .setTangent(Math.toRadians(45 * PoseHelper.allianceAngleMultiplier))
            .addTemporalMarker(this::outtake)
            .splineToLinearHeading(PoseHelper.backboardPose, Math.toRadians(0.0))
    }

    override fun toString() = "CloseInitToBackboardPathSegment"
}