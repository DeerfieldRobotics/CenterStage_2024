package org.firstinspires.ftc.teamcode.utils.auto.pathsegments

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper

class WhiteToParkPathSegment(override val robot: Robot) :
    RoadrunnerPathSegment(robot) {

    override var trajectorySequenceBuilder: TrajectorySequenceBuilder = robot.drive.trajectorySequenceBuilder(PoseHelper.currentPose)

    override fun buildPathSegment() {
        trajectorySequenceBuilder = trajectorySequenceBuilder
            .setVelConstraint(PoseHelper.blastVelocityConstraint)
            .setAccelConstraint(PoseHelper.blastAccelerationConstraint)
            .addTemporalMarker {
            robot.backCameraPortal?.close();
            robot.frontCameraPortal?.close();
        }
        .addTemporalMarker(this::outtakeTransfer)
            .forward(1.5)
            .back(1.5)
            .addTemporalMarker(this::stopIntake)
            .addTemporalMarker(this::transfer)
            .splineToLinearHeading(PoseHelper.wingTruss, Math.toRadians(0.0))
            .splineToLinearHeading(PoseHelper.boardTruss, Math.toRadians(0.0))
            .splineToLinearHeading(PoseHelper.parkPose, Math.toRadians(0.0))
    }

}
