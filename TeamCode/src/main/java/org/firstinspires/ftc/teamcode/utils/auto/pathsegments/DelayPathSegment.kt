package org.firstinspires.ftc.teamcode.utils.auto.pathsegments

class DelayPathSegment(override var duration: Double): PathSegment{
    override var running: Boolean = false

    override fun followPathSegment() {
        running = true
        val startTime = System.currentTimeMillis()
        while(System.currentTimeMillis() - startTime < duration * 1000){ /*do nothing*/ }
        running = false
    }

    override fun toString() = "DelayPathSegment: $duration seconds"
}