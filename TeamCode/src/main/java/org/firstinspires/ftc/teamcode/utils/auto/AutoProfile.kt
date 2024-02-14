package org.firstinspires.ftc.teamcode.utils.auto

import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper.Path
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper.StartPosition
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.PathSegment

class AutoProfile() {
    //TODO add delays

    val path: ArrayList<PathSegment> = arrayListOf()

    private fun addPathSegment(pathSegment: PathSegment) {
        path.add(pathSegment)
    }

    class AutoProfileBuilder() {
        private val profile = AutoProfile()
        fun addPathSegment(pathSegment: PathSegment) = apply {
            profile.addPathSegment(pathSegment)
        }

        fun build() = profile
    }
}