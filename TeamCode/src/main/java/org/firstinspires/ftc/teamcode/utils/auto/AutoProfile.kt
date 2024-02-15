package org.firstinspires.ftc.teamcode.utils.auto

import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.EmptyPathSegment
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.PathSegment

class AutoProfile() {
    //TODO add delays

    val path: ArrayList<PathSegment> = arrayListOf()

    fun addPathSegment(pathSegment: PathSegment) {
        path.add(pathSegment)
    }

    class AutoProfileBuilder() {
        val profile = AutoProfile()
        fun addPathSegment(pathSegment: PathSegment) = apply {
            profile.addPathSegment(pathSegment)
        }

        fun build() = profile
    }

    fun getDuration(): Double {
        var duration = 0.0
        for (segment in path) {
            duration += segment.duration
        }
        return duration
    }

    fun removeEmptySegments() {
        for(segment in path)
            if(segment is EmptyPathSegment)
                path.remove(segment)
    }
}