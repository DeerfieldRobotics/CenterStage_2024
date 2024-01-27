package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.utils.detection.AllianceHelper;
import org.firstinspires.ftc.teamcode.utils.detection.ColorDetectionProcessor;

public final class PoseHelper {
    // RED POSES
    public final static Pose2d initCloseRed = new Pose2d(10.5, -63, Math.toRadians(90.0));
    public final static Pose2d initFarRed = new Pose2d(-39.5, -63, Math.toRadians(90.0));
    public final static Pose2d backboardLeftRed = new Pose2d(50, -29, Math.toRadians(180.0));
    public final static Pose2d backboardCenterRed = new Pose2d(50, -35, Math.toRadians(180.0));
    public final static Pose2d backboardRightRed = new Pose2d(50, -41, Math.toRadians(180.0));
    public final static Pose2d farSpikeRightRed = new Pose2d(-29, -36, Math.toRadians(50.0));
    public final static Pose2d farSpikeCenterRed = new Pose2d(-40.5,-37, Math.toRadians(90.0));
    public final static Pose2d farSpikeLeftRed = new Pose2d(-46.5, -39.5, Math.toRadians(80.0));
    public final static Pose2d closeSpikeRightRed = new Pose2d(32.5, -30, Math.toRadians(180));
    public final static Pose2d closeSpikeCenterRed = new Pose2d(20.5, -24, Math.toRadians(180));
    public final static Pose2d closeSpikeLeftRed = new Pose2d(10.5, -30, Math.toRadians(180));
    public final static Pose2d apriltagStackRed = new Pose2d(-54.5, -35, Math.toRadians(180.0));
    public final static Pose2d middleStackRed = new Pose2d(-57.5, -23, Math.toRadians(180.0));
    public final static Pose2d wingTrussOutsideRed = new Pose2d(-35, -59, Math.toRadians(180.0));
    public final static Pose2d boardTrussOutsideRed = new Pose2d(8, -59, Math.toRadians(180.0));
    public final static Pose2d wingTrussInsideRed = new Pose2d(-33, -11, Math.toRadians(180.0));
    public final static Pose2d boardTrussInsideRed = new Pose2d(17, -11, Math.toRadians(180.0));
    public final static Pose2d parkPoseInsideRed = new Pose2d(58.5, -11, Math.toRadians(180.0));
    public final static Pose2d parkPoseOutsideRed = new Pose2d(58.5, -59, Math.toRadians(180.0));;

    // BLUE POSES
    public final static Pose2d initCloseBlue = new Pose2d(16, 63, Math.toRadians(270));
    public final static Pose2d initFarBlue = new Pose2d(-33, 63, Math.toRadians(270));
    public final static Pose2d backboardLeftBlue = new Pose2d(50, 41, Math.toRadians(180.0));
    public final static Pose2d backboardCenterBlue = new Pose2d(50, 35, Math.toRadians(180.0));
    public final static Pose2d backboardRightBlue = new Pose2d(50, 29, Math.toRadians(180.0));
    public final static Pose2d farSpikeRightBlue = new Pose2d(-48.5, 39.5, Math.toRadians(-80.0));
    public final static Pose2d farSpikeCenterBlue = new Pose2d(-40.5,34.5, Math.toRadians(90.0));
    public final static Pose2d farSpikeLeftBlue = new Pose2d(-29, 36, Math.toRadians(-50.0));
    public final static Pose2d closeSpikeRightBlue = new Pose2d(10.5, 30, Math.toRadians(180));
    public final static Pose2d closeSpikeCenterBlue = new Pose2d(20.5, 24, Math.toRadians(180));
    public final static Pose2d closeSpikeLeftBlue = new Pose2d(32.5, 30, Math.toRadians(180));
    public final static Pose2d apriltagStackBlue = new Pose2d(-54.5, 35, Math.toRadians(180.0));
    public final static Pose2d middleStackBlue = new Pose2d(-57.5, 23, Math.toRadians(180.0));
    public final static Pose2d wingTrussOutsideBlue = new Pose2d(-35, 59, Math.toRadians(180.0));
    public final static Pose2d boardTrussOutsideBlue = new Pose2d(8, 59, Math.toRadians(180.0));
    public final static Pose2d wingTrussInsideBlue = new Pose2d(-33, 11, Math.toRadians(180.0));
    public final static Pose2d boardTrussInsideBlue = new Pose2d(17, 11, Math.toRadians(180.0));
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

    public static Pose2d initPose;
    public static Pose2d spikePose;


    public static void buildAuto(){
//        this.alliance = alliance;
//        this.startPos = startPos;

        switch(AllianceHelper.alliance) {
            case RED:
                stackPose = PoseHelper.apriltagStackRed;
                allianceAngleMultiplier = 1.0;
                switch(Paths.path) {
                    case OUTSIDE:
                        parkPose = PoseHelper.parkPoseOutsideRed;
                        wingTruss = PoseHelper.wingTrussOutsideRed;
                        boardTruss = PoseHelper.boardTrussOutsideRed;
                        break;
                    case INSIDE:
                        parkPose = PoseHelper.parkPoseInsideRed;
                        wingTruss = PoseHelper.wingTrussInsideRed;
                        boardTruss = PoseHelper.boardTrussInsideRed;
                        break;
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
                stackPose = PoseHelper.apriltagStackBlue;
                allianceAngleMultiplier = -1.0;
                switch(Paths.path) {
                    case OUTSIDE:
                        parkPose = PoseHelper.parkPoseOutsideBlue;
                        wingTruss = PoseHelper.wingTrussOutsideBlue;
                        boardTruss = PoseHelper.boardTrussOutsideBlue;
                        break;
                    case INSIDE:
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
                        spikePose = PoseHelper.farSpikeLeftRed;
                        break;
                    case CENTER:
                        spikePose = PoseHelper.farSpikeCenterRed;
                        break;
                    case RIGHT:
                        spikePose = PoseHelper.farSpikeRightRed;
                        break;
                }
                break;
            case BLUE_CLOSE:
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
                        spikePose = PoseHelper.farSpikeLeftBlue;
                        break;
                    case CENTER:
                        spikePose = PoseHelper.farSpikeCenterBlue;
                        break;
                    case RIGHT:
                        spikePose = PoseHelper.farSpikeRightBlue;
                        break;
                }
                break;
        }
    }
}
