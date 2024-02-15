package org.firstinspires.ftc.teamcode.utils.auto.pathsegments

class EmptyPathSegment: PathSegment {
    override var running: Boolean = false
    override var duration: Double = 0.0

    override fun followPathSegment() {
        running = true
        running = false
    }
    override fun toString() = "EmptyPathSegment"
}