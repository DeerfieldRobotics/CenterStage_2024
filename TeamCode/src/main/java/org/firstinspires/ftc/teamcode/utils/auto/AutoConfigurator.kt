package org.firstinspires.ftc.teamcode.utils.auto

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.Telemetry
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
    private val gamepad1: Gamepad,
    private val gamepad2: Gamepad,
    robot: Robot
) {
    enum class PRESETS {
        RED_CLOSE_IN_2P4,
        RED_CLOSE_IN_2P2P2,
        RED_FAR_OUT_2P3,
        BLUE_CLOSE_IN_2P4,
        BLUE_CLOSE_IN_2P2P2,
        BLUE_FAR_OUT_2P3,
    }

    private val profileMap = mapOf(
        PRESETS.RED_CLOSE_IN_2P4 to AutoProfile.AutoProfileBuilder(
            PoseHelper.StartPosition.RED_CLOSE,
            PoseHelper.Path.INSIDE
        ).addPathSegment(CloseInitPathSegment(robot))
            .addPathSegment(RelocalizeToWhitePathSegment(robot))
            .addPathSegment(WhiteToBackboardPathSegment(robot, 0))
            .addPathSegment(ApriltagAlignToBackboardPathSegment(robot, false))
            .addPathSegment(BackboardToRelocalizePathSegment(robot))
            .addPathSegment(RelocalizeToWhitePathSegment(robot))
            .addPathSegment(WhiteToBackboardPathSegment(robot, 1))
            .addPathSegment(ApriltagAlignToBackboardPathSegment(robot, true))
            .addPathSegment(BackboardToParkPathSegment(robot))
            .build(),
        PRESETS.RED_CLOSE_IN_2P2P2 to AutoProfile.AutoProfileBuilder(
            PoseHelper.StartPosition.RED_CLOSE,
            PoseHelper.Path.INSIDE
        ).addPathSegment(CloseInitPathSegment(robot))
            .addPathSegment(RelocalizeToWhitePathSegment(robot))
            .addPathSegment(WhiteToBackboardPathSegment(robot, 0))
            .addPathSegment(ApriltagAlignToBackboardPathSegment(robot, false))
            .addPathSegment(BackboardToRelocalizePathSegment(robot))
            .addPathSegment(RelocalizeToWhitePathSegment(robot))
            .addPathSegment(WhiteToParkPathSegment(robot))
            .build(),
        PRESETS.RED_FAR_OUT_2P3 to AutoProfile.AutoProfileBuilder(
            PoseHelper.StartPosition.RED_FAR,
            PoseHelper.Path.OUTSIDE
        )
            .addPathSegment(FarInitPathSegment(robot))
            .addPathSegment(ApriltagAlignToBackboardPathSegment(robot, true))
            .addPathSegment(DropYellowPathSegment(robot))
            .addPathSegment(BackboardToRelocalizePathSegment(robot))
            .addPathSegment(RelocalizeToWhitePathSegment(robot))
            .addPathSegment(WhiteToBackboardPathSegment(robot, 0))
            .addPathSegment(ApriltagAlignToBackboardPathSegment(robot, true))
            .addPathSegment(BackboardToParkPathSegment(robot))
            .build(),
        PRESETS.BLUE_CLOSE_IN_2P4 to AutoProfile.AutoProfileBuilder(
            PoseHelper.StartPosition.BLUE_CLOSE,
            PoseHelper.Path.INSIDE
        ).addPathSegment(CloseInitPathSegment(robot))
            .addPathSegment(RelocalizeToWhitePathSegment(robot))
            .addPathSegment(WhiteToBackboardPathSegment(robot, 0))
            .addPathSegment(ApriltagAlignToBackboardPathSegment(robot, false))
            .addPathSegment(BackboardToRelocalizePathSegment(robot))
            .addPathSegment(RelocalizeToWhitePathSegment(robot))
            .addPathSegment(WhiteToBackboardPathSegment(robot, 1))
            .addPathSegment(ApriltagAlignToBackboardPathSegment(robot, true))
            .addPathSegment(BackboardToParkPathSegment(robot))
            .build(),
        PRESETS.BLUE_CLOSE_IN_2P2P2 to AutoProfile.AutoProfileBuilder(
            PoseHelper.StartPosition.BLUE_CLOSE,
            PoseHelper.Path.INSIDE
        ).addPathSegment(CloseInitPathSegment(robot))
            .addPathSegment(RelocalizeToWhitePathSegment(robot))
            .addPathSegment(WhiteToBackboardPathSegment(robot, 0))
            .addPathSegment(ApriltagAlignToBackboardPathSegment(robot, false))
            .addPathSegment(BackboardToRelocalizePathSegment(robot))
            .addPathSegment(RelocalizeToWhitePathSegment(robot))
            .addPathSegment(WhiteToParkPathSegment(robot))
            .build(),
        PRESETS.BLUE_FAR_OUT_2P3 to AutoProfile.AutoProfileBuilder(
            PoseHelper.StartPosition.BLUE_FAR,
            PoseHelper.Path.OUTSIDE
        )
            .addPathSegment(FarInitPathSegment(robot))
            .addPathSegment(ApriltagAlignToBackboardPathSegment(robot, true))
            .addPathSegment(DropYellowPathSegment(robot))
            .addPathSegment(BackboardToRelocalizePathSegment(robot))
            .addPathSegment(RelocalizeToWhitePathSegment(robot))
            .addPathSegment(WhiteToBackboardPathSegment(robot, 0))
            .addPathSegment(ApriltagAlignToBackboardPathSegment(robot, true))
            .addPathSegment(BackboardToParkPathSegment(robot))
            .build(),
    )

    private lateinit var autoProfile: AutoProfile

    fun configureAuto(): AutoProfile {
        configureStartingPosition()
        configurePath()
        telemetry.clear()
        return autoProfile
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
            var optionIndex = 0

            telemetry.addLine("             [15118 AUTO INITIALIZED]")
            telemetry.addLine("-------------------------------------------------")
            telemetry.addLine(" Selected ${PoseHelper.startPosition} Starting Position.")
            telemetry.addLine()
            telemetry.addLine("Select Autonomous Path and press CROSS (X) to confirm.")

            when (PoseHelper.startPosition) {
                PoseHelper.StartPosition.RED_CLOSE -> {
                    for (preset in PRESETS.entries)
                        if (preset.toString().contains("RED_CLOSE")) {
                            telemetryOption(preset.toString(), optionIndex == selectedPresetIndex)
                            autoProfile = profileMap[preset]!!
                            optionIndex++
                        }
                }

                PoseHelper.StartPosition.RED_FAR -> {
                    for (preset in PRESETS.entries)
                        if (preset.toString().contains("RED_FAR")) {
                            telemetryOption(preset.toString(), optionIndex == selectedPresetIndex)
                            autoProfile = profileMap[preset]!!
                            optionIndex++
                        }
                }

                PoseHelper.StartPosition.BLUE_CLOSE -> {
                    for (preset in PRESETS.entries)
                        if (preset.toString().contains("BLUE_CLOSE")) {
                            telemetryOption(preset.toString(), optionIndex == selectedPresetIndex)
                            autoProfile = profileMap[preset]!!
                            optionIndex++
                        }
                }

                PoseHelper.StartPosition.BLUE_FAR -> {
                    for (preset in PRESETS.entries)
                        if (preset.toString().contains("BLUE_FAR")) {
                            telemetryOption(preset.toString(), optionIndex == selectedPresetIndex)
                            autoProfile = profileMap[preset]!!
                            optionIndex++
                        }
                }

                else -> { throw Exception("Invalid Start Position") }
            }

            telemetry.update()

            if (gamepad1.left_stick_y < -0.5 || gamepad2.left_stick_y < -0.5 || gamepad1.dpad_down || gamepad2.dpad_down)
                selectedPresetIndex = (selectedPresetIndex - 1) % 4

            if (gamepad1.left_stick_y > 0.5 || gamepad2.left_stick_y > 0.5 || gamepad1.dpad_up || gamepad2.dpad_up)
                selectedPresetIndex = (selectedPresetIndex + 1) % 4

            if(gamepad1.cross || gamepad2.cross) break
        }
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