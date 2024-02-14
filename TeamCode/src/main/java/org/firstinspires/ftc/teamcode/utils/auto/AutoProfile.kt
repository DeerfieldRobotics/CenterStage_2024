package org.firstinspires.ftc.teamcode.utils.auto

import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper.Path
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper.StartPosition
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.PathSegment

class AutoProfile(startPosition: StartPosition, path: Path) {
    //TODO add delays
    init {
        PoseHelper.path = path
        PoseHelper.startPosition = startPosition

        PoseHelper.buildAuto()
    }

    val path: ArrayList<PathSegment> = arrayListOf()

    private fun addPathSegment(pathSegment: PathSegment) {
        path.add(pathSegment)
    }

    class AutoProfileBuilder(startPosition: StartPosition, path: Path) {
        private val profile = AutoProfile(startPosition, path)
        fun addPathSegment(pathSegment: PathSegment) = apply {
            profile.addPathSegment(pathSegment)
        }

        fun build() = profile
    }
}