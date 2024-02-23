package org.firstinspires.ftc.teamcode.utils.auto.pathsegments

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper

class BackboardToRelocalizePathSegment(
    override val robot: Robot
) : RoadrunnerPathSegment(robot) {
    override lateinit var trajectorySequenceBuilder: TrajectorySequenceBuilder

    override fun buildPathSegment() {
        trajectorySequenceBuilder = robot.drive.trajectorySequenceBuilder(PoseHelper.currentPose)
            .back(PoseHelper.backboardBackup)
            .addTemporalMarker(this::drop)
            .waitSeconds(0.4)
            .addTemporalMarker(this::outtakeIn)
            .addTemporalMarker { setSlideHeight(-1200) }
            .splineToConstantHeading(PoseHelper.aprilTruss.vec(), Math.toRadians(180.0))
            .waitSeconds(0.3)
    }

    override fun toString() = "BackboardToRelocalizePathSegment"
}