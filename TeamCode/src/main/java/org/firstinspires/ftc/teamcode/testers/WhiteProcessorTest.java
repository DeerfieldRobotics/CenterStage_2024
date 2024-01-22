package org.firstinspires.ftc.teamcode.testers;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.names.BuiltinCameraNameImpl;
import org.firstinspires.ftc.teamcode.roadrunner.drive.CogchampDrive;
import org.firstinspires.ftc.teamcode.utils.detection.WhiteDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "White Processor Test", group = "Testers")
@Config
public class WhiteProcessorTest extends LinearOpMode {
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.0;
    private VisionPortal visionPortal;
    private WhiteDetectionProcessor whiteProcessor;
//    private CogchampDrive drive;
    private CameraName camera;

    @Override
    public void runOpMode() throws InterruptedException {

        BuiltinCameraName camera = BuiltinCameraNameImpl.forCameraDirection(BuiltinCameraDirection.BACK);
//        camera = hardwareMap.get(WebcamName.class, "Webcam 2");

//        drive = new CogchampDrive(hardwareMap);
        whiteProcessor = new WhiteDetectionProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(camera)
                .setCameraResolution(new Size(320, 240))
                .addProcessor(whiteProcessor)
                .build();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            whiteProcessor.getController().setPID(kP, kI, kD);
            whiteProcessor.update();

//            if(gamepad1.left_bumper)
//                whiteProcessor.alignRobot(drive);
//            else {
//                double speedMult = .7 + 0.3 * gamepad1.right_trigger - 0.3 * gamepad1.left_trigger;
//
//                gamepad1.rumble(gamepad1.left_trigger > 0.5 ? (gamepad1.left_trigger - 0.5) / .4 : 0.0, gamepad1.right_trigger > 0.4 ? (gamepad1.right_trigger - 0.4) / 0.8 : 0.0, 50);
//
//                double forwardMult = 1;
//                double turnMult = .75;
//                double strafeMult = 1;
//
//                double forward = -gamepad1.left_stick_y * forwardMult * speedMult;
//                double turn = -gamepad1.right_stick_x * turnMult * speedMult;
//                double strafe = -gamepad1.left_stick_x * strafeMult * speedMult;
//
//                drive.setWeightedDrivePower(
//                        new Pose2d(
//                                forward,
//                                strafe,
//                                turn
//                        )
//                );
//            }

            telemetry.addData("White", whiteProcessor.getPosition());
            telemetry.addData("error", whiteProcessor.getError());
            telemetry.addData("power", whiteProcessor.getPower());
            telemetry.update();
        }
    }
}
