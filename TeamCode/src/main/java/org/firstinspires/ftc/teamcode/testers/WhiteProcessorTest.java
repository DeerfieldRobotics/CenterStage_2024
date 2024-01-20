package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.drive.CogchampDrive;
import org.firstinspires.ftc.teamcode.utils.detection.WhiteDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "White Processor Test", group = "Testers")
@Config
public class WhiteProcessorTest extends LinearOpMode {
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        CameraName camera = hardwareMap.get(WebcamName.class, "Webcam 1");

        CogchampDrive drive = new CogchampDrive(hardwareMap);
        WhiteDetectionProcessor whiteProcessor = new WhiteDetectionProcessor();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(camera)
                .addProcessor(whiteProcessor)
                .build();

        waitForStart();

        while(opModeIsActive()) {
            whiteProcessor.getController().setPID(kP, kI, kD);

            if(gamepad1.left_bumper)
                whiteProcessor.alignRobot(drive);

            telemetry.addData("White", whiteProcessor.getPosition());
            telemetry.update();
        }
    }
}
