package org.firstinspires.ftc.teamcode.utils.auto.pathsegments

import org.firstinspires.ftc.teamcode.roadrunner.drive.CogchampDrive
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper
import org.firstinspires.ftc.teamcode.utils.detection.AllianceHelper
import org.firstinspires.ftc.teamcode.utils.hardware.Intake
import org.firstinspires.ftc.teamcode.utils.hardware.Outtake
import org.firstinspires.ftc.teamcode.utils.hardware.Slide

class DropYellowPathSegment(
    override val robot: Robot
) : RoadrunnerPathSegment(robot) {
    override var trajectorySequenceBuilder: TrajectorySequenceBuilder =
        robot.drive.trajectorySequenceBuilder(PoseHelper.currentPose)

    override fun buildPathSegment() {
        trajectorySequenceBuilder = trajectorySequenceBuilder.back(PoseHelper.backboardBackup)
            .addTemporalMarker(this::drop)
            .waitSeconds(0.4)
            .addTemporalMarker { setSlideHeight(-1200) }
            .forward(PoseHelper.backboardBackup)
            .addTemporalMarker(this::outtakeTransfer)
            .waitSeconds(0.8)
            .addTemporalMarker(this::transfer)
            .waitSeconds(1.6)
            .addTemporalMarker(this::outtake)
            .addTemporalMarker { setSlideHeight(-1400) }
            .addTemporalMarker {
                //DEFAULTS TO RIGHT SIDE IF POSSIBLE
                if (PoseHelper.backboardPose == PoseHelper.backboardRightRed || PoseHelper.backboardPose == PoseHelper.backboardLeftBlue) {
                    if (AllianceHelper.alliance == AllianceHelper.Alliance.RED)
                        PoseHelper.backboardPose = PoseHelper.backboardCenterRed;
                    else
                        PoseHelper.backboardPose = PoseHelper.backboardCenterBlue;
                } else {
                    if (AllianceHelper.alliance == AllianceHelper.Alliance.RED)
                        PoseHelper.backboardPose = PoseHelper.backboardRightRed;
                    else
                        PoseHelper.backboardPose = PoseHelper.backboardLeftBlue;
                }
            }
    }
}