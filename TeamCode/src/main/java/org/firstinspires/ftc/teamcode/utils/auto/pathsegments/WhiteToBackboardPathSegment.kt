package org.firstinspires.ftc.teamcode.utils.auto.pathsegments

import org.firstinspires.ftc.teamcode.roadrunner.drive.CogchampDrive
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper
import org.firstinspires.ftc.teamcode.utils.detection.AllianceHelper
import org.firstinspires.ftc.teamcode.utils.hardware.Intake
import org.firstinspires.ftc.teamcode.utils.hardware.Outtake
import org.firstinspires.ftc.teamcode.utils.hardware.Slide

class WhiteToBackboardPathSegment(
    drive: CogchampDrive,
    intake: Intake,
    slide: Slide,
    outtake: Outtake,
    private val cycle: Int
) : RoadrunnerPathSegment(drive, intake, slide, outtake) {
    override var trajectorySequenceBuilder: TrajectorySequenceBuilder =
        drive.trajectorySequenceBuilder(
            PoseHelper.currentPose
        )

    override fun buildPathSegment() {
        trajectorySequenceBuilder = trajectorySequenceBuilder
            .setVelConstraint(PoseHelper.toBackboardVelocityConstraint)
            .setAccelConstraint(PoseHelper.toBackboardAccelerationConstraint)
            .addTemporalMarker(this::outtakeTransfer)
            .forward(1.5)
            .back(3.0)
            .addTemporalMarker(this::stopIntake)
            .addTemporalMarker(this::transfer)
            .splineToLinearHeading(PoseHelper.wingTruss, Math.toRadians(0.0))
            .splineToLinearHeading(PoseHelper.boardTruss, Math.toRadians(0.0))
            .addTemporalMarker(this::outtake)
            .addTemporalMarker { setSlideHeight(-1500) }
            .splineToLinearHeading(
                if (AllianceHelper.alliance == AllianceHelper.Alliance.RED) PoseHelper.backboardCenterRed else PoseHelper.backboardCenterBlue,
                Math.toRadians(if (PoseHelper.path == PoseHelper.Path.OUTSIDE) 30.0 else -30.0 * PoseHelper.allianceAngleMultiplier)
            )
            .addTemporalMarker {
                if (PoseHelper.path == PoseHelper.Path.INSIDE && cycle == 0) {
                    if (PoseHelper.backboardPose == PoseHelper.backboardLeftRed || PoseHelper.backboardPose == PoseHelper.backboardRightBlue) {
                        if (AllianceHelper.alliance == AllianceHelper.Alliance.RED)
                            PoseHelper.backboardPose = PoseHelper.backboardCenterRed;
                        else
                            PoseHelper.backboardPose = PoseHelper.backboardCenterBlue;
                    } else {
                        if (AllianceHelper.alliance == AllianceHelper.Alliance.RED)
                            PoseHelper.backboardPose = PoseHelper.backboardLeftRed;
                        else
                            PoseHelper.backboardPose = PoseHelper.backboardRightBlue;
                    }
                }
            }
    }
}