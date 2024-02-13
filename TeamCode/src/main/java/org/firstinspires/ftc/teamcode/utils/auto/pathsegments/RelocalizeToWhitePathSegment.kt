package org.firstinspires.ftc.teamcode.utils.auto.pathsegments

import org.firstinspires.ftc.teamcode.roadrunner.drive.CogchampDrive
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper
import org.firstinspires.ftc.teamcode.utils.hardware.Intake
import org.firstinspires.ftc.teamcode.utils.hardware.Outtake
import org.firstinspires.ftc.teamcode.utils.hardware.Slide

class RelocalizeToWhitePathSegment(
    drive: CogchampDrive,
    intake: Intake,
    slide: Slide,
    outtake: Outtake
) : RoadrunnerPathSegment(drive, intake, slide, outtake) {
    override var trajectorySequenceBuilder: TrajectorySequenceBuilder =
        drive.trajectorySequenceBuilder(
            PoseHelper.currentPose
        )

    override fun buildPathSegment() {
        trajectorySequenceBuilder = trajectorySequenceBuilder
            .addTemporalMarker {
                if (PoseHelper.path == PoseHelper.Path.OUTSIDE)
                    intake.servoPosition = Intake.IntakePositions.THREE
                else {
                    intake.servoPosition = Intake.IntakePositions.TWO
                }
                intake.update();
            }
            .setTangent(Math.toRadians(if (PoseHelper.path == PoseHelper.Path.INSIDE) 140.0 else -145.0) * PoseHelper.allianceAngleMultiplier)
            .splineToConstantHeading(PoseHelper.wingTruss.vec(), Math.toRadians(180.0))
            .addTemporalMarker(this::intake)
            .splineToConstantHeading(
                PoseHelper.stackPose.plus(PoseHelper.stackOffset).vec(),
                Math.toRadians(if (PoseHelper.path == PoseHelper.Path.INSIDE) 180.0 else 120.0 * PoseHelper.allianceAngleMultiplier)
            )
    }
}