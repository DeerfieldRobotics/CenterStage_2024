package org.firstinspires.ftc.teamcode.utils.auto.pathsegments

import android.util.Log
import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.utils.Other.LogcatHelper
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper

abstract class ApriltagPathSegment(open val robot: Robot) : PathSegment {
    fun aprilTagRelocalize() {
        robot.aprilTagProcessorBack?.update()
        Log.d(LogcatHelper.TAG, "AprilTag Pose: " + (robot.aprilTagProcessorBack?.poseEstimate))
        Log.d(LogcatHelper.TAG, "Drive Pose" + robot.drive.poseEstimate)
        if (!robot.aprilTagProcessorBack?.poseEstimate?.x?.isNaN()!!) {
            robot.drive.poseEstimate = Pose2d(
                robot.drive.poseEstimate.x,
                robot.aprilTagProcessorBack?.poseEstimate?.y!!,
                robot.aprilTagProcessorBack?.poseEstimate?.heading!!
            )
        }
        Log.d(LogcatHelper.TAG, "Set Pose" + robot.drive.poseEstimate)
    }

    fun alignToApriltagBackboard() {
        robot.aprilTagProcessorBack?.targetPose = PoseHelper.backboardPose
        val currentTime: Double = System.currentTimeMillis().toDouble()
        while (true) {
            robot.aprilTagProcessorBack?.update()
            if (System.currentTimeMillis() - currentTime > 750) break
            robot.aprilTagProcessorBack?.alignRobot(robot.drive)
            robot.intake.update()
            robot.slide.update()
            robot.outtake.update()
        }
        apriltagToDrivePose()
    }

    fun apriltagTuckerCarlson() {
        robot.aprilTagProcessorBack?.setPIDCoefficients(
            .042,
            .038,
            0.0,
            .030,
            .012,
            0.0,
            0.82,
            0.02,
            0.0
        )
    }

    private fun apriltagToDrivePose() {
        Log.d(LogcatHelper.TAG, "AprilTag Pose: " + (robot.aprilTagProcessorBack?.poseEstimate))
        Log.d(LogcatHelper.TAG, "Drive Pose" + robot.drive.poseEstimate)
        if (!robot.aprilTagProcessorBack?.poseEstimate?.x?.isNaN()!!) {
            robot.drive.poseEstimate = robot.aprilTagProcessorBack?.poseEstimate!!
        }
        PoseHelper.currentPose = robot.drive.poseEstimate
        Log.d(LogcatHelper.TAG, "Set Pose" + robot.drive.poseEstimate)
    }
}