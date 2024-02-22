package org.firstinspires.ftc.teamcode.utils.auto.pathsegments

import org.firstinspires.ftc.teamcode.utils.Robot

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

    override fun toString() = "ApriltagAlignToBackboardPathSegment"
}