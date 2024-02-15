package org.firstinspires.ftc.teamcode.utils.auto.pathsegments

import org.firstinspires.ftc.teamcode.utils.Robot

class ApriltagRelocalizePathSegment(robot: Robot) : ApriltagPathSegment(robot) {
    override var running: Boolean = false
    override var duration = 0.1
    override fun followPathSegment() {
        running = true
        aprilTagRelocalize()
        running = false
    }

    override fun toString() = "ApriltagRelocalizePathSegment"
}