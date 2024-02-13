package org.firstinspires.ftc.teamcode.utils.auto.pathsegments

import org.firstinspires.ftc.teamcode.roadrunner.drive.CogchampDrive
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper
import org.firstinspires.ftc.teamcode.utils.detection.AprilTagAlignmentProcessor
import org.firstinspires.ftc.teamcode.utils.hardware.Intake
import org.firstinspires.ftc.teamcode.utils.hardware.Outtake
import org.firstinspires.ftc.teamcode.utils.hardware.Slide

class ApriltagAlignToBackboardPathSegment(
    override val drive: CogchampDrive,
    override val intake: Intake,
    override val slide: Slide,
    override val outtake: Outtake,
    override val aprilTagAlignmentProcessor: AprilTagAlignmentProcessor,
    private val tuckerCarlson: Boolean
) : ApriltagPathSegment(drive, intake, slide, outtake, aprilTagAlignmentProcessor) {
    override var running: Boolean = false
    override fun followPathSegment() {
        running = true
        if (tuckerCarlson) apriltagTuckerCarlson()
        alignToApriltagBackboard()
        running = false
    }
}