package org.firstinspires.ftc.teamcode.utils.detection

import com.arcrobotics.ftclib.controller.PIDController
import org.firstinspires.ftc.teamcode.utils.hardware.Drivetrain
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
import java.util.TreeSet

class WhiteDetectionPipeline (private val drivetrain: Drivetrain?) : OpenCvPipeline() {
    private val threshold = 240
    private val whiteFrames: ArrayList<WhiteFrame> = ArrayList()
    private val startY = 120
    private val endY = 240
    private val startX = 0
    private val endX = 200
    private val numSectors = 10

    var target = 160
    var controller = PIDController(0.001, 0.0, 0.0)

    var position = -2.0

    init {
        controller.setTolerance(20.0)
    }

    override fun processFrame(input: Mat): Mat {
        val frame = WhiteFrame(numSectors, 3) //3 sector wide pixel image
        for (i in 0 until numSectors) frame.addSector(WhiteSector(0, i))

        //loop through each pixel in the frame and puts it in the correct sector
        for (i in startY until endY) {
            for (j in startX until endX) {
                if (input[i, j][0] > threshold && input[i, j][1] > threshold && input[i, j][2] > threshold) {
                    //increment at that index
                    frame.incrementSector(j * numSectors / endX)
                    frame.incrementXValue(j * numSectors / endX, j)
                }
            }
        }
        whiteFrames.add(frame)
        //5 averages
        while (whiteFrames.size > 5) {
            whiteFrames.removeAt(0)
        }

        //get the average over all frames
        var sum = 0.0
        for (f in whiteFrames) {
            if (f.whiteSectors.isNotEmpty()) sum += f.location.toDouble() //get that frame's average index
        }
        position = sum / whiteFrames.size

        // position line
        Imgproc.line(
            input,
            Point(position, startY.toDouble()),
            Point(position, endY.toDouble()),
            Scalar(255.0, 0.0, 0.0),
            1
        )

        //draw horizontal line
        Imgproc.line(
            input,
            Point(startX.toDouble(), startY.toDouble()),
            Point(endX.toDouble(), startY.toDouble()),
            Scalar(0.0, 0.0, 0.0),
            1
        )
        //draw lines on screen
        for (i in 0 until numSectors - 1) {
            Imgproc.line(
                input,
                Point(i.toDouble() * endX / numSectors, startY.toDouble()),
                Point(i.toDouble() * endX / numSectors, endY.toDouble()),
                if (whiteFrames.isNotEmpty() && whiteFrames[0].validSectors.isNotEmpty() && (whiteFrames[0].validSectors.contains(
                        i
                    ) || i != 0 && whiteFrames[0].validSectors.contains(i - 1))
                ) Scalar(0.0, 255.0, 0.0) else Scalar(0.0, 0.0, 0.0),
                1
            )
        }
        return input
    }
    fun alignRobot() {
        val error = target - position
        val power = controller.calculate(error)
        drivetrain?.move(0.0, power, 0.0)
    }
    fun robotAligned() = controller.atSetPoint()
    internal class WhiteFrame(
        private var maxSize: Int, //number of sectors to consider
        private var numSectors //pixel width in sectors
        : Int
    ) {
        val whiteSectors: ArrayList<WhiteSector> = ArrayList()
        val validSectors = ArrayList<Int>()
        fun addSector(sector: WhiteSector) {
            whiteSectors.add(sector)
            if (whiteSectors.size > maxSize) {
                whiteSectors.removeAt(0)
            }
        }
        fun incrementSector(index: Int) { whiteSectors[index].incrementWhiteCount() }

        fun incrementXValue(index: Int, x: Int) { whiteSectors[index].addXValue(x) }

        val location: Int
            get() {
                val xValues = ArrayList<Int>()
                val sortedSectors = TreeSet(whiteSectors)
                validSectors.clear()
                for (i in 0 until numSectors.coerceAtMost(sortedSectors.size)) {
                    val ws = sortedSectors.pollFirst()
                    xValues.addAll(ws.xValues)
                    validSectors.add(ws.index)
                }
                xValues.sort()
                return if (xValues.isNotEmpty()) xValues[xValues.size / 2] else -1
            }
    }

    internal class WhiteSector(var whiteCount: Int, val index: Int) : Comparable<WhiteSector?> {
        var xValues = ArrayList<Int>()
        fun incrementWhiteCount() { whiteCount++ }
        fun addXValue(x: Int) { xValues.add(x) }
        override fun compareTo(other: WhiteSector?): Int {
            return if (other != null) -whiteCount + other.whiteCount else 0
        }
    }
}