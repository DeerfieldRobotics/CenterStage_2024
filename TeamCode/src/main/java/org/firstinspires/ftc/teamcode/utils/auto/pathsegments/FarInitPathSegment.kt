package org.firstinspires.ftc.teamcode.utils.auto.pathsegments

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper
import org.firstinspires.ftc.teamcode.utils.detection.AllianceHelper
import org.firstinspires.ftc.teamcode.utils.hardware.Intake

class FarInitPathSegment(
    override val robot: Robot
) :
    RoadrunnerPathSegment(robot) {

    override lateinit var trajectorySequenceBuilder: TrajectorySequenceBuilder

    override fun buildPathSegment() {
        trajectorySequenceBuilder = robot.drive.trajectorySequenceBuilder(PoseHelper.initPose)
            .setTangent(PoseHelper.initialFarTangent * PoseHelper.allianceAngleMultiplier)
            .addTemporalMarker { robot.intake.servoPosition = Intake.IntakePositions.INTAKE }
            .splineToLinearHeading(PoseHelper.spikePose, PoseHelper.spikePose.heading)
            .addTemporalMarker(::outtakePurple)
            .back(PoseHelper.purpleBackDistanceFar)
            .addTemporalMarker { robot.intake.servoPosition = Intake.IntakePositions.FIVE }
            .setTangent(Math.toRadians(PoseHelper.toWhiteStackTangentFar))
            .splineToLinearHeading(
                PoseHelper.stackPose,
                Math.toRadians(PoseHelper.toWhiteStackTangentFar)
            )
            .addTemporalMarker(::intake)
            .forward(2.0)
            .back(2.0)
            .addTemporalMarker(::stopIntake)
            .splineToLinearHeading(PoseHelper.wingTruss, Math.toRadians(0.0))
            .addTemporalMarker { robot.intake.boosterServoPower = 0.0 }
            .splineToLinearHeading(PoseHelper.boardTruss, Math.toRadians(0.0))
            .addTemporalMarker(::outtake)
            .addTemporalMarker { setSlideHeight(-1050) }
            .splineToLinearHeading(
                if (AllianceHelper.alliance == AllianceHelper.Alliance.RED)
                    PoseHelper.backboardCenterRed
                else
                    PoseHelper.backboardCenterBlue, Math.toRadians(
                    if (PoseHelper.path == PoseHelper.Path.OUTSIDE)
                        30.0
                    else
                        -30.0 * PoseHelper.allianceAngleMultiplier
                )
            )
    }

    override fun toString() = "FarInitPathSegment"
}