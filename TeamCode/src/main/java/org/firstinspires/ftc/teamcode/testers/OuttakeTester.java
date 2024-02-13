package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.hardware.Outtake;
import org.firstinspires.ftc.teamcode.utils.hardware.Slide;

@TeleOp(name = "Outtake Tester", group = "c")
public class OuttakeTester extends LinearOpMode {
    private Slide slide;
    private Outtake outtake;
    private final boolean crossToggle = false;
    private double armAngle;
    private double wristAngle;
    @Override
    public void runOpMode() throws InterruptedException {
        slide = new Slide(hardwareMap);
        outtake = new Outtake(hardwareMap, slide);
        armAngle = outtake.getOuttakeAngle()[0];
        wristAngle = outtake.getOuttakeAngle()[1];
        waitForStart();
        while (opModeIsActive()) {
            outtake.setOuttakeKinematics(armAngle, wristAngle, true);
            armAngle += gamepad2.right_stick_y*.1;
            wristAngle += gamepad2.left_stick_y*.1;
            telemetry.addData("arm Angle", outtake.getOuttakeAngle()[0]);
            telemetry.addData("wrist Angle", outtake.getOuttakeAngle()[1]);
            telemetry.update();
        }
    }
}
