package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.utils.detection.AllianceHelper;
import org.firstinspires.ftc.teamcode.utils.detection.ColorDetectionProcessor;

public final class PoseHelper {
    // RED POSES
    public final static Pose2d initCloseRed = new Pose2d(10.5, -63, Math.toRadians(90.0));
    public final static Pose2d initFarRed = new Pose2d(-39.5, -63, Math.toRadians(90.0));
    public final static Pose2d backboardLeftRed = new Pose2d(48, -26.5, Math.toRadians(180.0));
    public final static Pose2d backboardCenterRed = new Pose2d(48, -31.5, Math.toRadians(180.0));
    public final static Pose2d backboardRightRed = new Pose2d(48, -37.5, Math.toRadians(180.0));
    public final static Pose2d farSpikeRightRed = new Pose2d(-30.5, -39, Math.toRadians(50.0));
    public final static Pose2d farSpikeCenterRed = new Pose2d(-39.5,-37, Math.toRadians(90.0));
    public final static Pose2d farSpikeLeftRed = new Pose2d(-46.5, -38.5, Math.toRadians(90.0));
    public final static Pose2d closeSpikeRightRed = new Pose2d(34.5, -28, Math.toRadians(180));
    public final static Pose2d closeSpikeCenterRed = new Pose2d(20.5, -20, Math.toRadians(180));
    public final static Pose2d closeSpikeLeftRed = new Pose2d(12.5, -30, Math.toRadians(180));
    public final static Pose2d apriltagStackRed = new Pose2d(-55.25, -36, Math.toRadians(180.0));
    public final static Pose2d middleStackRed = new Pose2d(-54.5, -24, Math.toRadians(180.0));
    public final static Pose2d insideStackRed = new Pose2d(-55.5, -5, Math.toRadians(180.0));
    public final static Pose2d wingTrussOutsideRed = new Pose2d(-35, -58, Math.toRadians(180.0));
    public final static Pose2d boardTrussOutsideRed = new Pose2d(8, -58, Math.toRadians(180.0));
    public final static Pose2d wingTrussInsideRed = new Pose2d(-33, -8, Math.toRadians(180.0));
    public final static Pose2d boardTrussInsideRed = new Pose2d(17, -8, Math.toRadians(180.0));
    public final static Pose2d parkPoseInsideRed = new Pose2d(58.5, -11, Math.toRadians(180.0));
    public final static Pose2d parkPoseOutsideRed = new Pose2d(58.5, -59, Math.toRadians(180.0));;

    // BLUE POSES
    public final static Pose2d initCloseBlue = new Pose2d(16, 63, Math.toRadians(270));
    public final static Pose2d initFarBlue = new Pose2d(-33, 63, Math.toRadians(270));
    public final static Pose2d backboardLeftBlue = new Pose2d(48, 40, Math.toRadians(180.0));
    public final static Pose2d backboardCenterBlue = new Pose2d(48, 34, Math.toRadians(180.0));
    public final static Pose2d backboardRightBlue = new Pose2d(48, 28, Math.toRadians(180.0));
    public final static Pose2d farSpikeRightBlue = new Pose2d(-47.5, 38.5, Math.toRadians(-90.0));
    public final static Pose2d farSpikeCenterBlue = new Pose2d(-41.5,37, Math.toRadians(-90.0));
    public final static Pose2d farSpikeLeftBlue = new Pose2d(-32, 40, Math.toRadians(-50.0));
    public final static Pose2d closeSpikeRightBlue = new Pose2d(10.5, 30, Math.toRadians(180));
    public final static Pose2d closeSpikeCenterBlue = new Pose2d(20.5, 24, Math.toRadians(180));
    public final static Pose2d closeSpikeLeftBlue = new Pose2d(32.5, 30, Math.toRadians(180));
    public final static Pose2d apriltagStackBlue = new Pose2d(-55.0, 36, Math.toRadians(180.0));
    public final static Pose2d middleStackBlue = new Pose2d(-54.5, 24, Math.toRadians(180.0));
    public final static Pose2d insideStackBlue = new Pose2d(-54.5, 12, Math.toRadians(180.0));
    public final static Pose2d wingTrussOutsideBlue = new Pose2d(-35, 56, Math.toRadians(180.0));
    public final static Pose2d boardTrussOutsideBlue = new Pose2d(8, 56, Math.toRadians(180.0));
    public final static Pose2d wingTrussInsideBlue = new Pose2d(-33, 9, Math.toRadians(180.0));
    public final static Pose2d boardTrussInsideBlue = new Pose2d(17, 9, Math.toRadians(180.0));
    public final static Pose2d parkPoseInsideBlue = new Pose2d(58.5, 11, Math.toRadians(180.0));
    public final static Pose2d parkPoseOutsideBlue = new Pose2d(58.5, 59, Math.toRadians(180.0));;
//
//    public Enum alliance;
//    public Enum startPos;
//

    public static Pose2d stackPose;
    public static double allianceAngleMultiplier;
    public static Pose2d parkPose;
    public static Pose2d wingTruss;
    public static Pose2d boardTruss;
    public static Pose2d backboardPose;
    public static double purpleBackDistanceFar;
    public static double toWhiteStackTangentFar;

    public static Pose2d initPose;
    public static Pose2d spikePose;
    public static double initialFarTangent;


    public static void buildAuto(){
//        this.alliance = alliance;
//        this.startPos = startPos;

        switch(AllianceHelper.alliance) {
            case RED:
                allianceAngleMultiplier = 1.0;
                switch(Paths.path) {
                    case OUTSIDE:
                        stackPose = PoseHelper.apriltagStackRed;
                        parkPose = PoseHelper.parkPoseOutsideRed;
                        wingTruss = PoseHelper.wingTrussOutsideRed;
                        boardTruss = PoseHelper.boardTrussOutsideRed;
                        break;
                    case INSIDE:
                        stackPose = PoseHelper.insideStackRed;
                        parkPose = PoseHelper.parkPoseInsideRed;
                        wingTruss = PoseHelper.wingTrussInsideRed;
                        boardTruss = PoseHelper.boardTrussInsideRed;
                        break;
                }
                switch(ColorDetectionProcessor.position) {
                    case LEFT:
                        backboardPose = PoseHelper.backboardLeftRed;;
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
                        break;
                    case INSIDE:
                        stackPose = PoseHelper.insideStackBlue;
                        parkPose = PoseHelper.parkPoseInsideBlue;
                        wingTruss = PoseHelper.wingTrussInsideBlue;
                        boardTruss = PoseHelper.boardTrussInsideBlue;
                        break;
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
                        initialFarTangent = 270;
                        purpleBackDistanceFar = 5.0;
                        toWhiteStackTangentFar = 180;
                        spikePose = PoseHelper.farSpikeCenterBlue;
                        break;
                    case RIGHT:
                        initialFarTangent = 270;
                        purpleBackDistanceFar = 8.0;
                        toWhiteStackTangentFar = 135;
                        spikePose = PoseHelper.farSpikeRightBlue;
                        break;
                }
                break;
        }
    }
}
