package org.firstinspires.ftc.teamcode.utils.auto

import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper.Path
import org.firstinspires.ftc.teamcode.utils.auto.PoseHelper.StartPosition
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.PathSegment
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.RoadrunnerPathSegment

class AutoConfig (startPosition: StartPosition, path: Path) {
    //TODO add delays
    init {
        PoseHelper.path = path
        PoseHelper.startPosition = startPosition

        PoseHelper.buildAuto()
    }

    val path : ArrayList<PathSegment> = arrayListOf()

    fun addPathSegment(pathSegment: PathSegment) {
        path.add(pathSegment)
    }
}