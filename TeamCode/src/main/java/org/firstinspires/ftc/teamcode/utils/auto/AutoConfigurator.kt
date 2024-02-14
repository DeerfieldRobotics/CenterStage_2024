package org.firstinspires.ftc.teamcode.utils.auto

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.detection.AllianceHelper

class AutoConfigurator (val telemetry: Telemetry, private val gamepad1: Gamepad, private val gamepad2: Gamepad) {
    fun configureAuto() {
        configureStartingPosition()
        configureBasePath()
        configurePath()
        //TODO add configs
        telemetry.clear()
    }

    private fun configureStartingPosition() {
        while (true) {
            telemetry.addLine("             [15118 AUTO INITIALIZED]")
            telemetry.addLine("-------------------------------------------------")
            telemetry.addLine("Select Autonomous Starting Position using DPAD Keys")
            telemetry.addData("    Blue Close   ", "(^)")
            telemetry.addData("    Blue Far     ", "(v)")
            telemetry.addData("    Red Far      ", "(<)")
            telemetry.addData("    Red Close    ", "(>)")
            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                PoseHelper.startPosition = PoseHelper.StartPosition.BLUE_CLOSE
                AllianceHelper.alliance = AllianceHelper.Alliance.BLUE
                break
            }
            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                PoseHelper.startPosition = PoseHelper.StartPosition.BLUE_FAR
                AllianceHelper.alliance = AllianceHelper.Alliance.BLUE
                break
            }
            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                PoseHelper.startPosition = PoseHelper.StartPosition.RED_FAR
                AllianceHelper.alliance = AllianceHelper.Alliance.RED
                break
            }
            if (gamepad1.dpad_right || gamepad2.dpad_right) {
                PoseHelper.startPosition = PoseHelper.StartPosition.RED_CLOSE
                AllianceHelper.alliance = AllianceHelper.Alliance.RED
                break
            }
            telemetry.update()
        }
    }
    private fun configureBasePath() {
        while (true) {
            telemetry.addLine("             [15118 AUTO INITIALIZED]")
            telemetry.addLine("-------------------------------------------------")
            telemetry.addLine(" Selected " + PoseHelper.startPosition.toString() + " Starting Position.")
            telemetry.addLine()
            telemetry.addLine("Select Autonomous Path using Shape Buttons")
            telemetry.addData("     Inside      ", "(Triangle)")
            telemetry.addData("     Outside     ", "(Cross)")
            telemetry.addData("     Placement   ", "(Circle)")
            if (gamepad1.triangle || gamepad2.triangle) {
                PoseHelper.path = PoseHelper.Path.INSIDE
                break
            }
            if (gamepad1.cross || gamepad2.cross) {
                PoseHelper.path = PoseHelper.Path.OUTSIDE
                break
            }
            if (gamepad1.circle || gamepad2.circle) {
                PoseHelper.path = PoseHelper.Path.PLACEMENT
                break
            }
            telemetry.update()
        }
    }

    private fun configurePath() {
        when(PoseHelper.path) {
            PoseHelper.Path.INSIDE -> {
                configureInsidePath()
            }
            PoseHelper.Path.OUTSIDE -> {
                configureOutsidePath()
            }
            PoseHelper.Path.PLACEMENT -> {
                configurePlacementPath()
            }
            else -> {
                configureBasePath()
                configurePath()
            }
        }
    }

    private fun configureInsidePath() {
        //TODO
    }

    private fun configureOutsidePath() {
        //TODO
    }

    private fun configurePlacementPath() {
        //TODO
    }
}