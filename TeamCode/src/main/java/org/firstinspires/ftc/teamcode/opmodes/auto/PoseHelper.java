package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.utils.detection.AllianceHelper;
import org.firstinspires.ftc.teamcode.utils.detection.ColorDetectionProcessor;

import java.util.Arrays;

public final class PoseHelper {
    // RED POSES
    public final static Pose2d initCloseRed = new Pose2d(10.5, -63, Math.toRadians(90.0));
    public final static Pose2d initFarRed = new Pose2d(-39.5, -63, Math.toRadians(90.0));
    public final static Pose2d backboardLeftRed = new Pose2d(48, -29.5, Math.toRadians(180.0));
    public final static Pose2d backboardCenterRed = new Pose2d(48, -35.5, Math.toRadians(180.0));
    public final static Pose2d backboardRightRed = new Pose2d(48, -41.5, Math.toRadians(180.0));
    public final static Pose2d farSpikeRightRed = new Pose2d(-30.5, -39, Math.toRadians(50.0));
    public final static Pose2d farSpikeCenterRed = new Pose2d(-39.5,-37, Math.toRadians(90.0));
    public final static Pose2d farSpikeLeftRed = new Pose2d(-46.5, -38.5, Math.toRadians(90.0));
    public final static Pose2d closeSpikeRightRed = new Pose2d(34.5, -33, Math.toRadians(180));
    public final static Pose2d closeSpikeCenterRed = new Pose2d(20.5, -22.5, Math.toRadians(180));
    public final static Pose2d closeSpikeLeftRed = new Pose2d(12, -30, Math.toRadians(180));
    public final static Pose2d apriltagStackRed = new Pose2d(-55.25, -36, Math.toRadians(180.0));
    public final static Pose2d middleStackRed = new Pose2d(-54.5, -24, Math.toRadians(180.0));
    public final static Pose2d insideStackRed = new Pose2d(-55, -8.0, Math.toRadians(180.0));
    public final static Pose2d wingTrussOutsideRed = new Pose2d(-35, -57, Math.toRadians(180.0));
    public final static Pose2d boardTrussOutsideRed = new Pose2d(12, -57, Math.toRadians(180.0));
    public final static Pose2d aprilTrussOutsideRed = new Pose2d(30, -52, Math.toRadians(180.0));
    public final static Pose2d wingTrussInsideRed = new Pose2d(-33, -7.5, Math.toRadians(180.0));
    public final static Pose2d boardTrussInsideRed = new Pose2d(12, -7.5, Math.toRadians(180.0));
    public final static Pose2d aprilTrussInsideRed = new Pose2d(31, -19, Math.toRadians(180.0));
    public final static Pose2d parkPoseInsideRed = new Pose2d(58.5, -11, Math.toRadians(180.0));
    public final static Pose2d parkPoseOutsideRed = new Pose2d(58.5, -59, Math.toRadians(180.0));

    // BLUE POSES
    public final static Pose2d initCloseBlue = new Pose2d(16, 63, Math.toRadians(270));
    public final static Pose2d initFarBlue = new Pose2d(-33, 63, Math.toRadians(270));
    public final static Pose2d backboardLeftBlue = new Pose2d(48.25, 41.5, Math.toRadians(180.0));
    public final static Pose2d backboardCenterBlue = new Pose2d(48.25, 35.5, Math.toRadians(180.0));
    public final static Pose2d backboardRightBlue = new Pose2d(48.25, 29.5, Math.toRadians(180.0));
    public final static Pose2d farSpikeRightBlue = new Pose2d(-46.5, 38.5, Math.toRadians(-90.0));
    public final static Pose2d farSpikeCenterBlue = new Pose2d(-39.5,37, Math.toRadians(-90.0));
    public final static Pose2d farSpikeLeftBlue = new Pose2d(-30.5, 38.5, Math.toRadians(-50.0));
    public final static Pose2d closeSpikeRightBlue = new Pose2d(12, 33, Math.toRadians(180));
    public final static Pose2d closeSpikeCenterBlue = new Pose2d(20.5, 26, Math.toRadians(180));
    public final static Pose2d closeSpikeLeftBlue = new Pose2d(33, 33, Math.toRadians(180));
    public final static Pose2d apriltagStackBlue = new Pose2d(-55.25, 36, Math.toRadians(180.0));
    public final static Pose2d middleStackBlue = new Pose2d(-54.5, 24, Math.toRadians(180.0));
    public final static Pose2d insideStackBlue = new Pose2d(-56, 16, Math.toRadians(180.0));
    public final static Pose2d wingTrussOutsideBlue = new Pose2d(-35, 56.75, Math.toRadians(180.0));
    public final static Pose2d boardTrussOutsideBlue = new Pose2d(8, 56.75, Math.toRadians(180.0));
    public final static Pose2d aprilTrussOutsideBlue = new Pose2d(30, 52, Math.toRadians(180.0));
    public final static Pose2d wingTrussInsideBlue = new Pose2d(-33, 10, Math.toRadians(180.0));
    public final static Pose2d boardTrussInsideBlue = new Pose2d(20, 12, Math.toRadians(180.0));
    public final static Pose2d aprilTrussInsideBlue = new Pose2d(31, 19, Math.toRadians(180.0));
    public final static Pose2d parkPoseInsideBlue = new Pose2d(58.5, 6, Math.toRadians(180.0));
    public final static Pose2d parkPoseOutsideBlue = new Pose2d(58.5, 59, Math.toRadians(180.0));


    public final static double backboardBackup = 5.25;
    public final static Pose2d stackOffset = new Pose2d(3, 1.5, 0);
    public final static TrajectoryVelocityConstraint toBackboardVelocityConstraint = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(5),
                new MecanumVelocityConstraint(45, TRACK_WIDTH)
        ));

    public final static TrajectoryVelocityConstraint blastVelocityConstraint = new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(5),
            new MecanumVelocityConstraint(62, TRACK_WIDTH)
    ));

    public final static TrajectoryVelocityConstraint toPurpleVelocityConstraint = new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(5),
            new MecanumVelocityConstraint(62, TRACK_WIDTH)
    ));

   public final static TrajectoryAccelerationConstraint toBackboardAccelerationConstraint = new ProfileAccelerationConstraint(45);

    public final static TrajectoryAccelerationConstraint blastAccelerationConstraint = new ProfileAccelerationConstraint(45);
    public static Pose2d stackPose;
    public static double allianceAngleMultiplier;
    public static Pose2d parkPose;
    public static Pose2d wingTruss;
    public static Pose2d boardTruss;
    public static Pose2d aprilTruss;
    public static Pose2d backboardPose;
    public static double purpleBackDistanceFar;
    public static double toWhiteStackTangentFar;

    public static Pose2d initPose;
    public static Pose2d spikePose;
    public static double initialFarTangent;

    public static Pose2d currentPose;


    public static void buildAuto(){
        switch(AllianceHelper.alliance) {
            case RED:
                allianceAngleMultiplier = 1.0;
                switch(Paths.path) {
                    case OUTSIDE:
                        stackPose = PoseHelper.apriltagStackRed;
                        parkPose = PoseHelper.parkPoseOutsideRed;
                        wingTruss = PoseHelper.wingTrussOutsideRed;
                        boardTruss = PoseHelper.boardTrussOutsideRed;
                        aprilTruss = PoseHelper.aprilTrussOutsideRed;
                        break;
                    case INSIDE:
                        stackPose = PoseHelper.insideStackRed;
                        parkPose = PoseHelper.parkPoseInsideRed;
                        wingTruss = PoseHelper.wingTrussInsideRed;
                        boardTruss = PoseHelper.boardTrussInsideRed;
                        aprilTruss = PoseHelper.aprilTrussInsideRed;
                        break;
                    case PLACEMENT:
                        parkPose = PoseHelper.parkPoseOutsideRed;
                }
                switch(ColorDetectionProcessor.position) {
                    case LEFT:
                        backboardPose = PoseHelper.backboardLeftRed;
                        break;
                    case CENTER:
                        backboardPose = PoseHelper.backboardCenterRed;
                        break;
                    case RIGHT:
                        backboardPose = PoseHelper.backboardRightRed;
                        break;
                }
                break;
            case BLUE:
                allianceAngleMultiplier = -1.0;
                switch(Paths.path) {
                    case OUTSIDE:
                        stackPose = PoseHelper.apriltagStackBlue;
                        parkPose = PoseHelper.parkPoseOutsideBlue;
                        wingTruss = PoseHelper.wingTrussOutsideBlue;
                        boardTruss = PoseHelper.boardTrussOutsideBlue;
                        aprilTruss = PoseHelper.aprilTrussOutsideBlue;
                        break;
                    case INSIDE:
                        stackPose = PoseHelper.insideStackBlue;
                        parkPose = PoseHelper.parkPoseInsideBlue;
                        wingTruss = PoseHelper.wingTrussInsideBlue;
                        boardTruss = PoseHelper.boardTrussInsideBlue;
                        aprilTruss = PoseHelper.aprilTrussInsideBlue;
                        break;
                    case PLACEMENT:
                        parkPose = PoseHelper.parkPoseOutsideBlue;
                }
                switch(ColorDetectionProcessor.position) {
                    case LEFT:
                        backboardPose = PoseHelper.backboardLeftBlue;
                        break;
                    case CENTER:
                        backboardPose = PoseHelper.backboardCenterBlue;
                        break;
                    case RIGHT:
                        backboardPose = PoseHelper.backboardRightBlue;
                        break;
                }
                break;
        }
        switch(StartPosition.startPosition) {
            case RED_CLOSE:
                toWhiteStackTangentFar = 180;
                initPose = PoseHelper.initCloseRed;
                switch(ColorDetectionProcessor.position) {
                    case LEFT:
                        spikePose = PoseHelper.closeSpikeLeftRed;
                        break;
                    case CENTER:
                        spikePose = PoseHelper.closeSpikeCenterRed;
                        break;
                    case RIGHT:
                        spikePose = PoseHelper.closeSpikeRightRed;
                        break;
                }
                break;
            case RED_FAR:
                initPose = PoseHelper.initFarRed;
                switch(ColorDetectionProcessor.position) {
                    case LEFT:
                        initialFarTangent = 90;
                        purpleBackDistanceFar = 8.0;
                        toWhiteStackTangentFar = 135;
                        spikePose = PoseHelper.farSpikeLeftRed;
                        break;
                    case CENTER:
                        initialFarTangent = 90;
                        purpleBackDistanceFar = 5.0;
                        toWhiteStackTangentFar = 180;
                        spikePose = PoseHelper.farSpikeCenterRed;
                        break;
                    case RIGHT:
                        initialFarTangent = 90;
                        purpleBackDistanceFar = 5.0;
                        toWhiteStackTangentFar = 180;
                        spikePose = PoseHelper.farSpikeRightRed;
                        break;
                }
                break;
            case BLUE_CLOSE:
                toWhiteStackTangentFar = 180;
                initPose = PoseHelper.initCloseBlue;
                switch(ColorDetectionProcessor.position) {
                    case LEFT:
                        spikePose = PoseHelper.closeSpikeLeftBlue;
                        break;
                    case CENTER:
                        spikePose = PoseHelper.closeSpikeCenterBlue;
                        break;
                    case RIGHT:
                        spikePose = PoseHelper.closeSpikeRightBlue;
                        break;
                }
                break;
            case BLUE_FAR:
                initPose = PoseHelper.initFarBlue;
                switch(ColorDetectionProcessor.position) {
                    case LEFT:
                        initialFarTangent = 60;
                        purpleBackDistanceFar = 5.0;
                        toWhiteStackTangentFar = 180;
                        spikePose = PoseHelper.farSpikeLeftBlue;
                        break;
                    case CENTER:
                        initialFarTangent = 90;
                        purpleBackDistanceFar = 5.0;
                        toWhiteStackTangentFar = 180;
                        spikePose = PoseHelper.farSpikeCenterBlue;
                        break;
                    case RIGHT:
                        initialFarTangent = 90;
                        purpleBackDistanceFar = 8.0;
                        toWhiteStackTangentFar = 135;
                        spikePose = PoseHelper.farSpikeRightBlue;
                        break;
                }
                break;
        }
    }
}
