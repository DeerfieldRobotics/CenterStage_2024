package org.firstinspires.ftc.teamcode.utils.auto.pathsegments

import com.acmerobotics.roadrunner.geometry.Pose2d
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
//            .setVelConstraint(PoseHelper.toPurpleVelocityConstraint)
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
//            .resetVelConstraint()
            .forward(3.5)
//            .waitSeconds(0.2)
//            .setVelConstraint(PoseHelper.defaultVelocityConstraint)
//            .back(2.0)
            .setTangent(Math.toRadians(355.0 /* * PoseHelper.allianceAngleMultiplier*/))
            .splineToLinearHeading(PoseHelper.wingTruss, Math.toRadians(0.0))
            .addTemporalMarker(::stopIntake)
            .addTemporalMarker { robot.intake.boosterServoPower = 0.0 }
            .setTangent(Math.toRadians(0.0))
            .splineToSplineHeading(PoseHelper.boardTruss, Math.toRadians(0.0))
//            .setTangent(Math.toRadians(0.0))
            .addTemporalMarker(::outtake)
            .addTemporalMarker { setSlideHeight(-1050) }
            .splineToSplineHeading(
                (if (AllianceHelper.alliance == AllianceHelper.Alliance.RED)
                    PoseHelper.backboardCenterRed
                else
                    PoseHelper.backboardCenterBlue).plus(
                        if(PoseHelper.path == PoseHelper.Path.OUTSIDE) {
                            Pose2d(0.0,8.0*PoseHelper.allianceAngleMultiplier, 0.0)
                        }
                        else {
                            Pose2d(0.0,0.0, 0.0)
                        }
                    )
                , Math.toRadians(
                    if (PoseHelper.path == PoseHelper.Path.OUTSIDE)
                        30.0
                    else
                        -30.0 * PoseHelper.allianceAngleMultiplier
                )
            )
    }

    override fun toString() = "FarInitPathSegment"
}