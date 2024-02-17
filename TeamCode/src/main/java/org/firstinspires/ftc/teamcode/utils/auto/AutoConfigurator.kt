package org.firstinspires.ftc.teamcode.utils.auto

import android.util.Log
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.utils.Other.LogcatHelper
import org.firstinspires.ftc.teamcode.utils.Robot
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.ApriltagAlignToBackboardPathSegment
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.ApriltagRelocalizePathSegment
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.BackboardToParkPathSegment
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.BackboardToRelocalizePathSegment
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.CloseInitPathSegment
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.DelayPathSegment
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.DropYellowPathSegment
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.EmptyPathSegment
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.FarInitPathSegment
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.PathSegment
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.RelocalizeToParkPathSegment
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.RelocalizeToWhitePathSegment
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.WhiteToBackboardPathSegment
import org.firstinspires.ftc.teamcode.utils.auto.pathsegments.WhiteToParkPathSegment
import org.firstinspires.ftc.teamcode.utils.detection.AllianceHelper

class AutoConfigurator(
    val telemetry: Telemetry,
    private var gamepad1: Gamepad,
    private var gamepad2: Gamepad,
    private val robot: Robot
) {
    enum class PRESETS {
        CLOSE_IN_2P4, CLOSE_IN_2P2P2, FAR_OUT_2P3, CUSTOM,
    }

    private val profileMap = mapOf(
        PRESETS.CLOSE_IN_2P4 to AutoProfile.AutoProfileBuilder()
            .addPathSegment(CloseInitPathSegment(robot))
            .addPathSegment(RelocalizeToWhitePathSegment(robot))
            .addPathSegment(WhiteToBackboardPathSegment(robot, 0))
            .addPathSegment(ApriltagAlignToBackboardPathSegment(robot, false))
            .addPathSegment(BackboardToRelocalizePathSegment(robot))
            .addPathSegment(ApriltagRelocalizePathSegment(robot))
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
            .addPathSegment(ApriltagRelocalizePathSegment(robot))
            .addPathSegment(RelocalizeToWhitePathSegment(robot))
            .addPathSegment(WhiteToParkPathSegment(robot)).build(),
        PRESETS.FAR_OUT_2P3 to AutoProfile.AutoProfileBuilder()
            .addPathSegment(FarInitPathSegment(robot))
            .addPathSegment(ApriltagAlignToBackboardPathSegment(robot, true))
            .addPathSegment(DropYellowPathSegment(robot))
            .addPathSegment(BackboardToRelocalizePathSegment(robot))
            .addPathSegment(ApriltagRelocalizePathSegment(robot))
            .addPathSegment(RelocalizeToWhitePathSegment(robot))
            .addPathSegment(WhiteToBackboardPathSegment(robot, 0))
            .addPathSegment(ApriltagAlignToBackboardPathSegment(robot, true))
            .addPathSegment(BackboardToParkPathSegment(robot)).build(),
        PRESETS.CUSTOM to AutoProfile.AutoProfileBuilder().build()
    )

    private var selectUp = false
    private var upToggle = false
    private var selectDown = false
    private var downToggle = false
    private var selectLeft = false
    private var leftToggle = false
    private var selectRight = false
    private var rightToggle = false

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
                        telemetryProfileOption(preset, optionIndex == selectedPresetIndex)
                        optionIndex++
                    }
                }

                PoseHelper.StartPosition.RED_FAR -> {
                    for (preset in PRESETS.entries) if (preset.toString().contains("FAR")) {
                        telemetryProfileOption(preset, optionIndex == selectedPresetIndex)
                        optionIndex++
                    }
                }

                PoseHelper.StartPosition.BLUE_CLOSE -> {
                    for (preset in PRESETS.entries) if (preset.toString().contains("CLOSE")) {
                        telemetryProfileOption(preset, optionIndex == selectedPresetIndex)
                        optionIndex++
                    }
                }

                PoseHelper.StartPosition.BLUE_FAR -> {
                    for (preset in PRESETS.entries) if (preset.toString().contains("FAR")) {
                        telemetryProfileOption(preset, optionIndex == selectedPresetIndex)
                        optionIndex++
                    }
                }

                else -> {
                    throw Exception("Invalid Start Position")
                }
            }

            //ALWAYS ADD OPTION FOR CUSTOM PATH
            telemetryProfileOption(PRESETS.CUSTOM, optionIndex == selectedPresetIndex)
            optionIndex++

            telemetry.update()

            if (selectDown && !downToggle) {
                selectedPresetIndex = (selectedPresetIndex - 1) % (optionIndex + 1)
                downToggle = true
            } else if (!selectDown) {
                downToggle = false
            }

            if (selectUp && !upToggle) {
                selectedPresetIndex = (selectedPresetIndex + 1) % (optionIndex + 1)
                upToggle = true
            } else if (!selectUp) {
                upToggle = false
            }

            if (gamepad1.cross || gamepad2.cross) break
        }

        //Checks if the selected path is inside or outside
        if (autoProfile == profileMap[PRESETS.CUSTOM]) {
            customPath()
        }

    }

    private fun customPath() {
        var selectedPresetIndex = 0
        while (true) {
            updateSelection()

            var optionIndex = 0

            telemetry.addLine("               [PATH CUSTOMIZATION]")
            telemetry.addLine("-------------------------------------------------")
            telemetry.addLine("Select Path Template and press CROSS (X) to confirm.")

            when (PoseHelper.startPosition) {
                PoseHelper.StartPosition.RED_CLOSE -> {
                    for (preset in PRESETS.entries) if (preset.toString().contains("CLOSE")) {
                        telemetryProfileOption(preset, optionIndex == selectedPresetIndex)
                        optionIndex++
                    }
                }

                PoseHelper.StartPosition.RED_FAR -> {
                    for (preset in PRESETS.entries) if (preset.toString().contains("FAR")) {
                        telemetryProfileOption(preset, optionIndex == selectedPresetIndex)
                        optionIndex++
                    }
                }

                PoseHelper.StartPosition.BLUE_CLOSE -> {
                    for (preset in PRESETS.entries) if (preset.toString().contains("CLOSE")) {
                        telemetryProfileOption(preset, optionIndex == selectedPresetIndex)
                        optionIndex++
                    }
                }

                PoseHelper.StartPosition.BLUE_FAR -> {
                    for (preset in PRESETS.entries) if (preset.toString().contains("FAR")) {
                        telemetryProfileOption(preset, optionIndex == selectedPresetIndex)
                        optionIndex++
                    }
                }

                else -> {
                    throw Exception("Invalid Start Position")
                }
            }
            telemetry.update()

            if (selectDown && !downToggle) {
                selectedPresetIndex = (selectedPresetIndex - 1) % (optionIndex + 1)
                downToggle = true
            } else if (!selectDown) {
                downToggle = false
            }

            if (selectUp && !upToggle) {
                selectedPresetIndex = (selectedPresetIndex + 1) % (optionIndex + 1)
                upToggle = true
            } else if (!selectUp) {
                upToggle = false
            }

            if (gamepad1.cross || gamepad2.cross) break
        }
        editPath()
    }

    private fun editPath() {
        var selectedSegmentIndex = 0

        //SET UP CUSTOM PATH WITH EMPTY PATHS THAT WILL BE REMOVED LATER
        val customProfileBuilder = AutoProfile.AutoProfileBuilder()
        customProfileBuilder.addPathSegment(EmptyPathSegment())
        for (segment in autoProfile.path)
            customProfileBuilder.addPathSegment(segment).addPathSegment(EmptyPathSegment())

        while (true) {
            updateSelection()

            telemetry.addLine("               [PATH CUSTOMIZATION]")
            telemetry.addLine("-------------------------------------------------")
            telemetry.addLine("Select Segment to edit using CROSS (X). Press CIRCLE (O) to confirm.")

            for ((segmentIndex, segment) in customProfileBuilder.profile.path.withIndex())
                telemetryPathSegmentOption(segment, segmentIndex == selectedSegmentIndex)

            if (selectDown) selectedSegmentIndex =
                (selectedSegmentIndex - 1) % customProfileBuilder.profile.path.size
            if (selectUp) selectedSegmentIndex =
                (selectedSegmentIndex + 1) % customProfileBuilder.profile.path.size

            if (gamepad1.cross || gamepad2.cross) {
                editPathSegment(selectedSegmentIndex)
                break
            }

            if (gamepad1.circle || gamepad2.circle) {
                customProfileBuilder.profile.removeEmptySegments()
                autoProfile = customProfileBuilder.build()
                break
            }

            telemetry.update()
        }
    }

    private fun editPathSegment(segmentIndex: Int) {
        while (true) {
            telemetry.addLine("            [SEGMENT CUSTOMIZATION]")
            telemetry.addLine("-------------------------------------------------")
            telemetry.addLine("Press SQUARE ([]) to remove segment. Press CIRCLE (O) to change segment.")

            if (gamepad1.square || gamepad2.square) {
                autoProfile.path.removeAt(segmentIndex)
                editPath()
                break
            }
            if (gamepad1.circle || gamepad2.circle) {
                changePathSegment(segmentIndex)
                break
            }
        }
    }

    private fun changePathSegment(segmentIndex: Int) {
        var selectedSegmentIndex = 0
        val pathList = listOf(
            DelayPathSegment(0.0),
            EmptyPathSegment(),
            CloseInitPathSegment(robot),
            FarInitPathSegment(robot),
            ApriltagRelocalizePathSegment(robot),
            ApriltagAlignToBackboardPathSegment(robot, true),
            BackboardToRelocalizePathSegment(robot),
            BackboardToParkPathSegment(robot),
            RelocalizeToWhitePathSegment(robot),
            RelocalizeToParkPathSegment(robot),
            WhiteToBackboardPathSegment(robot, 0),
            WhiteToParkPathSegment(robot),
            DropYellowPathSegment(robot)
        )

        while (true) {
            telemetry.addLine("            [SEGMENT CUSTOMIZATION]")
            telemetry.addLine("-------------------------------------------------")
            telemetry.addLine("Select the path to change to and press CROSS (X) to confirm.")

            for ((index, segment) in pathList.withIndex())
                telemetryPathSegmentOption(segment, index == selectedSegmentIndex)

            if (gamepad1.cross || gamepad2.cross) {
                autoProfile.path.removeAt(segmentIndex)
                if (selectedSegmentIndex != 0 && selectedSegmentIndex != 10)
                    autoProfile.path.add(segmentIndex, pathList[selectedSegmentIndex])
                else if (selectedSegmentIndex == 0) {
                    var duration = 3.0
                    while (true) {
                        telemetry.clear()
                        telemetry.addLine("            [SEGMENT CUSTOMIZATION]")
                        telemetry.addLine("-------------------------------------------------")
                        telemetry.addLine("Select the duration of the delay and press CROSS (X) to confirm.")
                        telemetry.addData("Duration: ", duration)

                        autoProfile.path.add(segmentIndex, DelayPathSegment(duration))

                        if (gamepad1.cross || gamepad2.cross) break

                        if (selectDown) duration -= 0.5
                        if (selectUp) duration += 0.5
                    }
                } else {
                    var cycles = 1
                    while (true) {
                        telemetry.clear()
                        telemetry.addLine("            [SEGMENT CUSTOMIZATION]")
                        telemetry.addLine("-------------------------------------------------")
                        telemetry.addLine("Select the number of cycles and press CROSS (X) to confirm.")
                        telemetry.addData("Duration: ", cycles)

                        autoProfile.path.add(
                            segmentIndex,
                            WhiteToBackboardPathSegment(robot, cycles)
                        )

                        if (gamepad1.cross || gamepad2.cross) break

                        if (selectDown) cycles -= 1
                        if (selectUp) cycles += 1
                    }
                }
                break
            }

            if (gamepad1.circle || gamepad2.circle) {
                val segment = autoProfile.path[segmentIndex]
                autoProfile.path.removeAt(segmentIndex)
                autoProfile.path.add(segmentIndex, segment)
                break
            }

            if (selectDown) selectedSegmentIndex = (selectedSegmentIndex - 1) % (pathList.size)
            if (selectUp) selectedSegmentIndex = (selectedSegmentIndex + 1) % (pathList.size)
        }
        editPath()
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

    private fun telemetryProfileOption(option: PRESETS, selected: Boolean) {
        var endSpace = ""
        for (i in 0 until 13 - option.toString().length) {
            endSpace += " "
        }
        if (selected) {
            telemetry.addLine("     $option$endSpace[*]")
            autoProfile = profileMap[option]!!
            if (option.toString().contains("IN")) PoseHelper.path = PoseHelper.Path.INSIDE
            else if (option.toString().contains("OUT")) PoseHelper.path = PoseHelper.Path.OUTSIDE
        } else {
            telemetry.addLine("     $option$endSpace[ ]")
        }
    }

    private fun telemetryPathSegmentOption(option: PathSegment, selected: Boolean) {
        var endSpace = ""
        for (i in 0 until 13 - option.toString().length) {
            endSpace += " "
        }
        if (selected) {
            telemetry.addLine("     $option$endSpace[*]")
        } else {
            telemetry.addLine("     $option$endSpace[ ]")
        }
    }
}