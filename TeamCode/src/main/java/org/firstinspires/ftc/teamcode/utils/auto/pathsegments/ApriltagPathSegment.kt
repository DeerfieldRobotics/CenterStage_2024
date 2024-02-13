package org.firstinspires.ftc.teamcode.utils.auto.pathsegments

import android.util.Log
import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.roadrunner.drive.CogchampDrive
import org.firstinspires.ftc.teamcode.utils.Other.LogcatHelper
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper
import org.firstinspires.ftc.teamcode.utils.detection.AprilTagAlignmentProcessor
import org.firstinspires.ftc.teamcode.utils.hardware.Intake
import org.firstinspires.ftc.teamcode.utils.hardware.Outtake
import org.firstinspires.ftc.teamcode.utils.hardware.Slide

abstract class ApriltagPathSegment(open val drive: CogchampDrive, open val intake: Intake, open val slide: Slide, open val outtake: Outtake, open val aprilTagAlignmentProcessor: AprilTagAlignmentProcessor): PathSegment {
    fun aprilTagRelocalize() {
        aprilTagAlignmentProcessor.update()
        Log.d(LogcatHelper.TAG, "AprilTag Pose: " + (aprilTagAlignmentProcessor.poseEstimate))
        Log.d(LogcatHelper.TAG, "Drive Pose" + drive.poseEstimate)
        if (!aprilTagAlignmentProcessor.poseEstimate.x.isNaN()) {
            drive.poseEstimate = Pose2d(
                drive.poseEstimate.x,
                aprilTagAlignmentProcessor.poseEstimate.y,
                aprilTagAlignmentProcessor.poseEstimate.heading
            )
        }
        Log.d(LogcatHelper.TAG, "Set Pose" + drive.poseEstimate)
    }

    fun alignToApriltagBackboard() {
        aprilTagAlignmentProcessor.targetPose = PoseHelper.backboardPose
        val currentTime: Double = System.currentTimeMillis().toDouble()
        while (true) {
            aprilTagAlignmentProcessor.update()
            if (System.currentTimeMillis() - currentTime > 750) break
            aprilTagAlignmentProcessor.alignRobot(drive)
            intake.update()
            slide.update()
            outtake.update()
        }
        apriltagToDrivePose()
    }
    fun apriltagTuckerCarlson() {
        aprilTagAlignmentProcessor.setPIDCoefficients(.042, .038, 0.0, .030, .012, 0.0, 0.82, 0.02, 0.0)
    }

    private fun apriltagToDrivePose() {
        Log.d(LogcatHelper.TAG, "AprilTag Pose: " + (aprilTagAlignmentProcessor.poseEstimate))
        Log.d(LogcatHelper.TAG, "Drive Pose" + drive.poseEstimate)
        if (!aprilTagAlignmentProcessor.poseEstimate.x.isNaN()) {
            drive.poseEstimate = aprilTagAlignmentProcessor.poseEstimate
        }
        PoseHelper.currentPose = drive.poseEstimate
        Log.d(LogcatHelper.TAG, "Set Pose" + drive.poseEstimate)
    }
}