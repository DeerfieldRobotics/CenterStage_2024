package org.firstinspires.ftc.teamcode.utils.auto.pathsegments

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper

class BackboardToParkPathSegment(
    override val robot: Robot
) : RoadrunnerPathSegment(robot) {
    override lateinit var trajectorySequenceBuilder: TrajectorySequenceBuilder

    override fun buildPathSegment() {
        trajectorySequenceBuilder = robot.drive.trajectorySequenceBuilder(PoseHelper.currentPose)
            .addTemporalMarker {
                //TODO close if fixed in next SDK
                robot.backCameraPortal?.stopStreaming()
            }
            .back(PoseHelper.backboardBackup)
            .addTemporalMarker(this::drop)
            .addTemporalMarker { setSlideHeight(-1600) }
            .waitSeconds(.4)
            .addTemporalMarker(this::outtakeIn)
            .forward(5.0)
            .strafeRight(22.0 * PoseHelper.allianceAngleMultiplier * (if (PoseHelper.path == PoseHelper.Path.INSIDE) 1.0 else -1.0))
            .back(10.0)
            .addTemporalMarker {
                outtakeTransfer(); //transfer just for fun
                transfer();
            }
    }

    override fun toString() = "BackboardToParkPathSegment"
}