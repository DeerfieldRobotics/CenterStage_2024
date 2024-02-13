package org.firstinspires.ftc.teamcode.utils.auto.pathsegments

import org.firstinspires.ftc.teamcode.roadrunner.drive.CogchampDrive
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper
import org.firstinspires.ftc.teamcode.utils.hardware.Intake
import org.firstinspires.ftc.teamcode.utils.hardware.Outtake
import org.firstinspires.ftc.teamcode.utils.hardware.Slide

class RelocalizeToParkPathSegment(
    drive: CogchampDrive,
    intake: Intake,
    slide: Slide,
    outtake: Outtake
) : RoadrunnerPathSegment(drive, intake, slide, outtake) {

    override var trajectorySequenceBuilder: TrajectorySequenceBuilder = drive.trajectorySequenceBuilder( PoseHelper.currentPose)

    override fun buildPathSegment() { //idk what this shit does
        trajectorySequenceBuilder = trajectorySequenceBuilder.setTangent(0.0)
            .strafeLeft(24*PoseHelper.allianceAngleMultiplier)
            .splineToLinearHeading(PoseHelper.parkPose, Math.toRadians(180.0))
            .addTemporalMarker {
            outtakeTransfer(); //transfer just for fun
            transfer();
        }
    }
}