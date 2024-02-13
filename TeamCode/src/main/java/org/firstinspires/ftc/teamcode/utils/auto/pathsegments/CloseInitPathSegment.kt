package org.firstinspires.ftc.teamcode.utils.auto.pathsegments

import org.firstinspires.ftc.teamcode.roadrunner.drive.CogchampDrive
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper
import org.firstinspires.ftc.teamcode.utils.detection.AllianceHelper
import org.firstinspires.ftc.teamcode.utils.detection.ColorDetectionProcessor
import org.firstinspires.ftc.teamcode.utils.hardware.Intake
import org.firstinspires.ftc.teamcode.utils.hardware.Outtake
import org.firstinspires.ftc.teamcode.utils.hardware.Slide

class CloseInitPathSegment(
    override val drive: CogchampDrive,
    override val intake: Intake,
    override val slide: Slide,
    override val outtake: Outtake
) :
    RoadrunnerPathSegment(drive, intake, slide, outtake) {

    override var trajectorySequenceBuilder: TrajectorySequenceBuilder =
        drive.trajectorySequenceBuilder(PoseHelper.initPose)

    override fun buildPathSegment() {
        trajectorySequenceBuilder = trajectorySequenceBuilder
            .setVelConstraint(PoseHelper.toBackboardVelocityConstraint)
            .setTangent(Math.toRadians(45 * PoseHelper.allianceAngleMultiplier))
            .addTemporalMarker(this::outtake)
            .splineToLinearHeading(PoseHelper.backboardPose, Math.toRadians(0.0))
            .setVelConstraint(PoseHelper.toPurpleVelocityConstraint)
            .back(PoseHelper.backboardBackup)
            .addTemporalMarker(this::drop)
            .waitSeconds(0.2)
            .addTemporalMarker { setSlideHeight(-1100) }
            .waitSeconds(0.2)
            .forward(PoseHelper.backboardBackup)
            .addTemporalMarker(this::outtakeIn)
            .addTemporalMarker { intake.servoPosition = Intake.IntakePositions.INTAKE }
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
            .addTemporalMarker { intake.servoPosition = Intake.IntakePositions.FOUR }
    }
}