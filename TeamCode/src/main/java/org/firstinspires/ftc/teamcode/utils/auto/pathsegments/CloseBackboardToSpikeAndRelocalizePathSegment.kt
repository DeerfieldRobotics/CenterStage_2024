package org.firstinspires.ftc.teamcode.utils.auto.pathsegments

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper
import org.firstinspires.ftc.teamcode.utils.detection.AllianceHelper
import org.firstinspires.ftc.teamcode.utils.detection.ColorDetectionProcessor
import org.firstinspires.ftc.teamcode.utils.hardware.Intake

class CloseBackboardToSpikeAndRelocalizePathSegment(
    override val robot: Robot
) :
    RoadrunnerPathSegment(robot) {

    override lateinit var trajectorySequenceBuilder: TrajectorySequenceBuilder

    override fun buildPathSegment() {
        trajectorySequenceBuilder = robot.drive.trajectorySequenceBuilder(PoseHelper.currentPose)
            .setVelConstraint(PoseHelper.toPurpleVelocityConstraint)
            .back(PoseHelper.backboardBackup)
            .addTemporalMarker(this::drop)
            .waitSeconds(0.2)
            .addTemporalMarker { setSlideHeight(-1100) }
            .forward(PoseHelper.backboardBackup)
            .addTemporalMarker(this::outtakeIn)
            .addTemporalMarker { robot.intake.servoPosition = Intake.IntakePositions.INTAKE }
            .setTangent(Math.toRadians(180.0))
            .splineToLinearHeading(PoseHelper.spikePose, Math.toRadians(180.0))
            .back(4.0)
            .addTemporalMarker(this::outtakePurple)
            .waitSeconds(0.1)
            .back(4.0)
            .setTangent(
                Math.toRadians(
                    (if ((ColorDetectionProcessor.position == ColorDetectionProcessor.StartingPosition.RIGHT && AllianceHelper.alliance == AllianceHelper.Alliance.RED) ||
                        (ColorDetectionProcessor.position == ColorDetectionProcessor.StartingPosition.LEFT && AllianceHelper.alliance == AllianceHelper.Alliance.BLUE)
                    ) 45.0 * PoseHelper.allianceAngleMultiplier else 0.0)
                )
            )
            .splineToConstantHeading(PoseHelper.aprilTruss.vec(), Math.toRadians(180.0))
            .addTemporalMarker { robot.intake.servoPosition = Intake.IntakePositions.FOUR }
            .waitSeconds(0.5)
    }

    override fun toString() = "CloseBackboardToSpikeAndRelocalizePathSegment"
}