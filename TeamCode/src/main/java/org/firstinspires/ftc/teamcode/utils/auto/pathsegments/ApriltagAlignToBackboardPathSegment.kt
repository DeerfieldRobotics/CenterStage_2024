package org.firstinspires.ftc.teamcode.utils.auto.pathsegments

import org.firstinspires.ftc.teamcode.roadrunner.drive.CogchampDrive
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper
import org.firstinspires.ftc.teamcode.utils.detection.AprilTagAlignmentProcessor
import org.firstinspires.ftc.teamcode.utils.hardware.Intake
import org.firstinspires.ftc.teamcode.utils.hardware.Outtake
import org.firstinspires.ftc.teamcode.utils.hardware.Slide

class ApriltagAlignToBackboardPathSegment(
    override val robot: Robot,
    private val tuckerCarlson: Boolean
) : ApriltagPathSegment(robot) {
    override var running: Boolean = false
    override var duration = 0.75
    override fun followPathSegment() {
        running = true
        if (tuckerCarlson) apriltagTuckerCarlson()
        alignToApriltagBackboard()
        running = false
    }
}