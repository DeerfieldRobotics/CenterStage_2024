package org.firstinspires.ftc.teamcode.utils.hardware

import com.arcrobotics.ftclib.hardware.ServoEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap

class Launcher (hardwareMap: HardwareMap) {
    private var loaded = 0.0
    private var fire = 0.06
    private var launcherServo: ServoImplEx = hardwareMap.get("ls") as ServoImplEx //control hub: 2
    init {
        launcherServo.position = loaded
    }
    fun fire() {
        launcherServo.position = fire
    }
    fun load() {
        launcherServo.position = loaded
    }
}