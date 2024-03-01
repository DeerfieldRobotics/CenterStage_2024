package org.firstinspires.ftc.teamcode.utils.auto.pathsegments

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper
import org.firstinspires.ftc.teamcode.utils.detection.AllianceHelper
import org.firstinspires.ftc.teamcode.utils.hardware.Intake

class RelocalizeToBackboardPathSegment(
    override val robot: Robot, private val cycle: Int
) : RoadrunnerPathSegment(robot) {
    override lateinit var trajectorySequenceBuilder: TrajectorySequenceBuilder

    override fun buildPathSegment() {
        trajectorySequenceBuilder = robot.drive.trajectorySequenceBuilder(PoseHelper.currentPose)
            .addTemporalMarker {
                if (PoseHelper.path == PoseHelper.Path.OUTSIDE)
                    robot.intake.servoPosition = Intake.IntakePositions.THREE
                else if (cycle == 0)
                    robot.intake.servoPosition = Intake.IntakePositions.FOUR
                else
                    robot.intake.servoPosition = Intake.IntakePositions.TWO
                robot.intake.update();
            }
            .setTangent(Math.toRadians(if (PoseHelper.path == PoseHelper.Path.INSIDE) 140.0 else -160.0) * PoseHelper.allianceAngleMultiplier)
            .splineToConstantHeading(PoseHelper.wingTruss.vec(), Math.toRadians(180.0))
            .addTemporalMarker(::intake)
        approachStack()
        trajectorySequenceBuilder = trajectorySequenceBuilder
//            .setVelConstraint(PoseHelper.toBackboardVelocityConstraint)
//            .setAccelConstraint(PoseHelper.toBackboardAccelerationConstraint)
            .addTemporalMarker(this::outtakeTransfer)
            .waitSeconds(0.1)
            .forward(4.5)
//            .waitSeconds(0.8)
            //TO BACKBOARD
            .setTangent(Math.toRadians(180.0) - PoseHelper.stackPose.heading)
            .UNSTABLE_addTemporalMarkerOffset(0.6) { stopIntake(); transfer() }
            .splineToSplineHeading(PoseHelper.wingTruss, Math.toRadians(0.0))
            .splineToSplineHeading(PoseHelper.boardTruss, Math.toRadians(0.0))
            .addTemporalMarker(this::outtake).addTemporalMarker { setSlideHeight(-1500) }
            .splineToSplineHeading(
                if (AllianceHelper.alliance == AllianceHelper.Alliance.RED) PoseHelper.backboardCenterRed else PoseHelper.backboardCenterBlue,
                Math.toRadians(if (PoseHelper.path == PoseHelper.Path.OUTSIDE) 30.0 else -30.0 * PoseHelper.allianceAngleMultiplier)
            )
            //RESET BACKBOARD POSE
            .addTemporalMarker {
                PoseHelper.backboardPose =
                if (PoseHelper.path == PoseHelper.Path.INSIDE && cycle == 0) {
                    if (PoseHelper.backboardPose == PoseHelper.backboardLeftRed || PoseHelper.backboardPose == PoseHelper.backboardRightBlue) {
                        if (AllianceHelper.alliance == AllianceHelper.Alliance.RED)
                            PoseHelper.backboardCenterRed
                        else PoseHelper.backboardCenterBlue
                    } else {
                        if (AllianceHelper.alliance == AllianceHelper.Alliance.RED)
                            PoseHelper.backboardLeftRed
                        else  PoseHelper.backboardRightBlue
                    }
                }
                else if (PoseHelper.path == PoseHelper.Path.OUTSIDE) {
                    if(AllianceHelper.alliance == AllianceHelper.Alliance.RED) {
                        if (PoseHelper.backboardPose == PoseHelper.backboardRightRed) {
                            PoseHelper.backboardCenterRed
                        } else {
                            PoseHelper.backboardRightRed
                        }
                    }
                    else {
                        if (PoseHelper.backboardPose == PoseHelper.backboardLeftBlue) {
                            PoseHelper.backboardCenterBlue
                        } else {
                            PoseHelper.backboardLeftBlue
                        }
                    }
                }
                else { //DEFAULT TO CENTER
                    if(AllianceHelper.alliance == AllianceHelper.Alliance.RED)
                        PoseHelper.backboardCenterRed
                    else
                        PoseHelper.backboardCenterBlue
                }
            }
    }

    override fun toString() = "RelocalizeToBackboardPathSegment"
}