package org.firstinspires.ftc.teamcode.utils.auto.pathsegments

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper

class RelocalizeToParkPathSegment(
    override val robot: Robot
) : RoadrunnerPathSegment(robot) {

    override var trajectorySequenceBuilder: TrajectorySequenceBuilder =
        robot.drive.trajectorySequenceBuilder(PoseHelper.currentPose)

    override fun buildPathSegment() { //idk what this shit does
        trajectorySequenceBuilder = trajectorySequenceBuilder.setTangent(0.0)
            .strafeLeft(24 * PoseHelper.allianceAngleMultiplier)
            .splineToLinearHeading(PoseHelper.parkPose, Math.toRadians(180.0))
            .addTemporalMarker {
                outtakeTransfer(); //transfer just for fun
                transfer();
            }
    }
}