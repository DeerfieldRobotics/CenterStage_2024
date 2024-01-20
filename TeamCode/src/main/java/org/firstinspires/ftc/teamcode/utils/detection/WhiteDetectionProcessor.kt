package org.firstinspires.ftc.teamcode.utils.detection

import android.graphics.Canvas
import android.graphics.Paint
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.arcrobotics.ftclib.controller.PIDController
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.teamcode.roadrunner.drive.CogchampDrive
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Mat
import java.util.TreeSet

class WhiteDetectionProcessor : VisionProcessor {
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
    override fun init(width: Int, height: Int, calibration: CameraCalibration) {}
    override fun processFrame(frame: Mat, captureTimeNanos: Long): Any? {
        val whiteFrame =
           WhiteFrame(numSectors, 3) //3 sector wide pixel image
        for (i in 0 until numSectors) whiteFrame.addSector(WhiteSector(0, i))

        //loop through each pixel in the frame and puts it in the correct sector
        for (i in startY until endY) {
            for (j in startX until endX) {
                if (frame[i, j][0] > threshold && frame[i, j][1] > threshold && frame[i, j][2] > threshold) {
                    //increment at that index
                    whiteFrame.incrementSector(j * numSectors / endX)
                    whiteFrame.incrementXValue(j * numSectors / endX, j)
                }
            }
        }
        whiteFrames.add(whiteFrame)
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
        return null
    }

    override fun onDrawFrame(
        canvas: Canvas,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any
    ) {
        canvas.drawLine(position.toFloat(), 0f, position.toFloat(), 240f, Paint());
    }
    internal class WhiteFrame(
        private var maxSize: Int, //number of sectors to consider
        private var numSectors //pixel width in sectors
        : Int
    ) {
        val whiteSectors: ArrayList<WhiteSector> = ArrayList()
        private val validSectors = ArrayList<Int>()
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

    fun alignRobot(drivetrain: CogchampDrive) {
        val error = target - position
        val power = controller.calculate(error)
        drivetrain.setWeightedDrivePower(Pose2d(0.0, power, 0.0))
    }
    fun robotAligned() = controller.atSetPoint()

    internal class WhiteSector(private var whiteCount: Int, val index: Int) : Comparable<WhiteSector?> {
        var xValues = ArrayList<Int>()
        fun incrementWhiteCount() { whiteCount++ }
        fun addXValue(x: Int) { xValues.add(x) }
        override fun compareTo(other: WhiteSector?): Int {
            return if (other != null) -whiteCount + other.whiteCount else 0
        }
    }
}