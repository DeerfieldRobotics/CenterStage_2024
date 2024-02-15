package org.firstinspires.ftc.teamcode.utils.auto.pathsegments

interface PathSegment {
    var running: Boolean
    var duration: Double
    fun followPathSegment()
    override fun toString(): String
}