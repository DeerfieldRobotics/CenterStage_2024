package org.firstinspires.ftc.teamcode.utils.auto

import android.util.Log
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.Other.LogcatHelper
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.ApriltagAlignToBackboardPathSegment
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.BackboardToParkPathSegment
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.BackboardToRelocalizePathSegment
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.CloseInitPathSegment
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.DropYellowPathSegment
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.FarInitPathSegment
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.RelocalizeToWhitePathSegment
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.WhiteToBackboardPathSegment
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.WhiteToParkPathSegment
import org.firstinspires.ftc.teamcode.utils.detection.AllianceHelper

class AutoConfigurator(
    val telemetry: Telemetry,
    private var gamepad1: Gamepad,
    private var gamepad2: Gamepad,
    robot: Robot
) {
    enum class PRESETS {
        CLOSE_IN_2P4, CLOSE_IN_2P2P2, FAR_OUT_2P3,
    }

    private val profileMap = mapOf(
        PRESETS.CLOSE_IN_2P4 to AutoProfile.AutoProfileBuilder()
            .addPathSegment(CloseInitPathSegment(robot))
            .addPathSegment(RelocalizeToWhitePathSegment(robot))
            .addPathSegment(WhiteToBackboardPathSegment(robot, 0))
            .addPathSegment(ApriltagAlignToBackboardPathSegment(robot, false))
            .addPathSegment(BackboardToRelocalizePathSegment(robot))
            .addPathSegment(RelocalizeToWhitePathSegment(robot))
            .addPathSegment(WhiteToBackboardPathSegment(robot, 1))
            .addPathSegment(ApriltagAlignToBackboardPathSegment(robot, true))
            .addPathSegment(BackboardToParkPathSegment(robot)).build(),
        PRESETS.CLOSE_IN_2P2P2 to AutoProfile.AutoProfileBuilder()
            .addPathSegment(CloseInitPathSegment(robot))
            .addPathSegment(RelocalizeToWhitePathSegment(robot))
            .addPathSegment(WhiteToBackboardPathSegment(robot, 0))
            .addPathSegment(ApriltagAlignToBackboardPathSegment(robot, false))
            .addPathSegment(BackboardToRelocalizePathSegment(robot))
            .addPathSegment(RelocalizeToWhitePathSegment(robot))
            .addPathSegment(WhiteToParkPathSegment(robot)).build(),
        PRESETS.FAR_OUT_2P3 to AutoProfile.AutoProfileBuilder()
            .addPathSegment(FarInitPathSegment(robot))
            .addPathSegment(ApriltagAlignToBackboardPathSegment(robot, true))
            .addPathSegment(DropYellowPathSegment(robot))
            .addPathSegment(BackboardToRelocalizePathSegment(robot))
            .addPathSegment(RelocalizeToWhitePathSegment(robot))
            .addPathSegment(WhiteToBackboardPathSegment(robot, 0))
            .addPathSegment(ApriltagAlignToBackboardPathSegment(robot, true))
            .addPathSegment(BackboardToParkPathSegment(robot)).build(),
    )

    private var selectUp = false
    private var selectDown = false
    private var selectLeft = false
    private var selectRight = false

    private lateinit var autoProfile: AutoProfile

    fun configureAuto(): AutoProfile {
        configureStartingPosition()
        configurePath()
        telemetry.clear()
        return autoProfile
    }

    private fun configureStartingPosition() {
        while (true) {
            updateSelection()

            Log.d(LogcatHelper.TAG, "Configuring Starting Position...")
            telemetry.addLine("             [15118 AUTO INITIALIZED]")
            telemetry.addLine("-------------------------------------------------")
            telemetry.addLine("Select Autonomous Starting Position using DPAD Keys")
            telemetry.addData("    Blue Close   ", "(^)")
            telemetry.addData("    Blue Far     ", "(v)")
            telemetry.addData("    Red Far      ", "(<)")
            telemetry.addData("    Red Close    ", "(>)")
            if (selectUp) {
                PoseHelper.startPosition = PoseHelper.StartPosition.BLUE_CLOSE
                AllianceHelper.alliance = AllianceHelper.Alliance.BLUE
                Log.d(LogcatHelper.TAG, "Blue Close Selected")
                break
            }
            if (selectDown) {
                PoseHelper.startPosition = PoseHelper.StartPosition.BLUE_FAR
                AllianceHelper.alliance = AllianceHelper.Alliance.BLUE
                Log.d(LogcatHelper.TAG, "Blue Far Selected")
                break
            }
            if (selectLeft) {
                PoseHelper.startPosition = PoseHelper.StartPosition.RED_FAR
                AllianceHelper.alliance = AllianceHelper.Alliance.RED
                Log.d(LogcatHelper.TAG, "Red Far Selected")
                break
            }
            if (selectRight) {
                PoseHelper.startPosition = PoseHelper.StartPosition.RED_CLOSE
                AllianceHelper.alliance = AllianceHelper.Alliance.RED
                Log.d(LogcatHelper.TAG, "Red Close Selected")
                break
            }
            telemetry.update()
        }
    }

    private fun configurePath() {
        //WAIT FOR RELEASE
        while (gamepad1.dpad_down || gamepad2.dpad_down || gamepad1.dpad_up || gamepad2.dpad_up || gamepad1.dpad_left || gamepad2.dpad_left || gamepad1.dpad_right || gamepad2.dpad_right) {
            telemetry.addLine("             [15118 AUTO INITIALIZED]")
            telemetry.addLine("-------------------------------------------------")
            telemetry.addLine(" Selected ${PoseHelper.startPosition} Starting Position.")
        }

        telemetry.clear()

        var selectedPresetIndex = 0

        while (true) {
            updateSelection()

            var optionIndex = 0

            telemetry.addLine("             [15118 AUTO INITIALIZED]")
            telemetry.addLine("-------------------------------------------------")
            telemetry.addLine(" Selected ${PoseHelper.startPosition} Starting Position.")
            telemetry.addLine()
            telemetry.addLine("Select Autonomous Path and press CROSS (X) to confirm.")

            when (PoseHelper.startPosition) {
                PoseHelper.StartPosition.RED_CLOSE -> {
                    for (preset in PRESETS.entries) if (preset.toString().contains("CLOSE")) {
                        telemetryOption(preset.toString(), optionIndex == selectedPresetIndex)
                        autoProfile = profileMap[preset]!!
                        optionIndex++
                    }
                }

                PoseHelper.StartPosition.RED_FAR -> {
                    for (preset in PRESETS.entries) if (preset.toString().contains("FAR")) {
                        telemetryOption(preset.toString(), optionIndex == selectedPresetIndex)
                        autoProfile = profileMap[preset]!!
                        optionIndex++
                    }
                }

                PoseHelper.StartPosition.BLUE_CLOSE -> {
                    for (preset in PRESETS.entries) if (preset.toString().contains("CLOSE")) {
                        telemetryOption(preset.toString(), optionIndex == selectedPresetIndex)
                        autoProfile = profileMap[preset]!!
                        optionIndex++
                    }
                }

                PoseHelper.StartPosition.BLUE_FAR -> {
                    for (preset in PRESETS.entries) if (preset.toString().contains("FAR")) {
                        telemetryOption(preset.toString(), optionIndex == selectedPresetIndex)
                        autoProfile = profileMap[preset]!!
                        optionIndex++
                    }
                }

                else -> {
                    throw Exception("Invalid Start Position")
                }
            }

            telemetry.update()

            if (selectDown) selectedPresetIndex =
                (selectedPresetIndex - 1) % optionIndex

            if (selectUp) selectedPresetIndex =
                (selectedPresetIndex + 1) % optionIndex

            if (gamepad1.cross || gamepad2.cross) break
        }

        //Checks if the selected path is inside or outside

        if (autoProfile.toString().contains("IN"))
            PoseHelper.path = PoseHelper.Path.INSIDE
        else (autoProfile.toString().contains("OUT"))
            PoseHelper.path = PoseHelper.Path.OUTSIDE
    }

    private fun updateSelection() {
        selectUp =
            gamepad1.dpad_up || gamepad2.dpad_up || gamepad1.left_stick_y > 0.5 || gamepad2.left_stick_y > 0.5
        selectDown =
            gamepad1.dpad_down || gamepad2.dpad_down || gamepad1.left_stick_y < -0.5 || gamepad2.left_stick_y < -0.5
        selectLeft =
            gamepad1.dpad_left || gamepad2.dpad_left || gamepad1.left_stick_x < -0.5 || gamepad2.left_stick_x < -0.5
        selectRight =
            gamepad1.dpad_right || gamepad2.dpad_right || gamepad1.left_stick_x > 0.5 || gamepad2.left_stick_x > 0.5
    }

    private fun telemetryOption(option: String, selected: Boolean) {
        var endSpace = ""
        for (i in 0 until 13 - option.length) {
            endSpace += " "
        }
        if (selected) {
            telemetry.addLine("     $option$endSpace[*]")
        } else {
            telemetry.addLine("     $option$endSpace[ ]")
        }
    }
}