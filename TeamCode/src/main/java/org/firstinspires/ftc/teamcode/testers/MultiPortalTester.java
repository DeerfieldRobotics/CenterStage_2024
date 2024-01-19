package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.detection.AllianceHelper;
import org.firstinspires.ftc.teamcode.utils.detection.AprilTagAlignmentAuto;
import org.firstinspires.ftc.teamcode.utils.detection.ColorDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl;

import java.util.List;
@Autonomous(name = "MultiPortalTester")
public class MultiPortalTester extends LinearOpMode {

    private AprilTagAlignmentAuto aprilTagAlignment;
    private VisionPortal aprilTagPortal;
    private VisionPortal colorPortal;
    private AprilTagProcessorImpl aprilTagProcessor;
    private ColorDetectionProcessor colorDetectionProcessor;

    @Override
    public void runOpMode() throws InterruptedException {
        AllianceHelper.alliance = AllianceHelper.Alliance.RED;
        CameraName backCamera = hardwareMap.get(WebcamName.class, "Webcam 1");
        CameraName frontCamera = hardwareMap.get(WebcamName.class, "Webcam 2");

        AprilTagLibrary aprilTagLibrary = new AprilTagLibrary.Builder()
                .addTag(1, "BlueLeft", 2.0, DistanceUnit.INCH)
                .addTag(2, "BlueCenter", 2.0, DistanceUnit.INCH)
                .addTag(3, "BlueRight", 2.0, DistanceUnit.INCH)
                .addTag(4, "RedLeft", 2.0, DistanceUnit.INCH)
                .addTag(5, "RedCenter", 2.0, DistanceUnit.INCH)
                .addTag(6, "RedRight", 2.0, DistanceUnit.INCH)
                .build();

        aprilTagProcessor = new AprilTagProcessorImpl(902.125, 902.125, 604.652, 368.362, DistanceUnit.INCH, AngleUnit.DEGREES, aprilTagLibrary, true, true, true, true, AprilTagProcessor.TagFamily.TAG_36h11, 1); // Used for managing the AprilTag detection process.
        colorDetectionProcessor = new ColorDetectionProcessor(AllianceHelper.Alliance.RED); // Used for managing the color detection process.

        List<Integer> myPortalList = JavaUtil.makeIntegerList(VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL));
        int portal_1_View_ID = (Integer) JavaUtil.inListGet(myPortalList, JavaUtil.AtMode.FROM_START, 0, false);
        int portal_2_View_ID = (Integer) JavaUtil.inListGet(myPortalList, JavaUtil.AtMode.FROM_START, 1, false);


        aprilTagPortal = new VisionPortal.Builder()
                .setCamera(backCamera)
                .setCameraResolution(new android.util.Size(1280, 720))
                .addProcessor(aprilTagProcessor)
                .setLiveViewContainerId(portal_2_View_ID)
                .build();
        aprilTagAlignment = new AprilTagAlignmentAuto(backCamera, 0.0, 12.0, 0.0, AllianceHelper.alliance, aprilTagProcessor);

        colorPortal = new VisionPortal.Builder()
                .setCamera(frontCamera)
                .setCameraResolution(new android.util.Size(320, 240))
                .addProcessor(colorDetectionProcessor)
                .setLiveViewContainerId(portal_1_View_ID)
                .build();

        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            aprilTagAlignment.update();
            telemetry.addData("X", aprilTagAlignment.getCurrentX());
            telemetry.addData("Y", aprilTagAlignment.getCurrentY());
            telemetry.addData("Heading", aprilTagAlignment.getCurrentHeading());
            telemetry.addData("Robot Aligned", aprilTagAlignment.robotAligned());
            telemetry.addData("Target X", aprilTagAlignment.getTargetX());
            telemetry.addData("Target Y", aprilTagAlignment.getTargetY());
            telemetry.addData("Target Heading", aprilTagAlignment.getTargetHeading());
            telemetry.addData("Color", colorDetectionProcessor.getPosition());
            telemetry.update();
        }
    }

}
