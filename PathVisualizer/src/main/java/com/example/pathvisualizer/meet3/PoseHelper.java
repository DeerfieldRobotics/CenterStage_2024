package com.example.pathvisualizer.meet3;

import com.acmerobotics.roadrunner.geometry.Pose2d;

abstract public class PoseHelper {
    // RED POSES
    public static Pose2d initCloseRed = new Pose2d(10.5, -63, Math.toRadians(90.0));
    public static Pose2d backboardRed = new Pose2d(50, -35, Math.toRadians(180.0));
    public static Pose2d initFarRed = new Pose2d(-39.5, -63, Math.toRadians(90.0));
    public static Pose2d farSpikeRightRed = new Pose2d(-29, -36, Math.toRadians(50.0));
    public static Pose2d farSpikeCenterRed = new Pose2d(-44.5,-34.5, Math.toRadians(90.0));
    public static Pose2d farSpikeLeftRed = new Pose2d(-48.5, -39.5, Math.toRadians(80.0));
    public static Pose2d closeSpikeRightRed = new Pose2d(32.5, -30, Math.toRadians(180));
    public static Pose2d closeSpikeCenterRed = new Pose2d(20.5, -24, Math.toRadians(180));
    public static Pose2d closeSpikeLeftRed = new Pose2d(10.5, -30, Math.toRadians(180));
    public static Pose2d apriltagStackRed = new Pose2d(-57.5, -35, Math.toRadians(180.0));
    public static Pose2d middleStackRed = new Pose2d(-57.5, -23, Math.toRadians(180.0));
    public static Pose2d wingTrussRed = new Pose2d(-35, -59, Math.toRadians(180.0));
    public static Pose2d boardTrussRed = new Pose2d(8, -59, Math.toRadians(180.0));

    // BLUE POSES


    public static Pose2d initCloseBlue = null; //TODO
    public static Pose2d backboardBlue = new Pose2d(50, 35, Math.toRadians(180.0));
    public static Pose2d initFarBlue = null; //TODO
    public static Pose2d farSpikeRightBlue = new Pose2d(-29, 36, Math.toRadians(50.0));
    public static Pose2d farSpikeCenterBlue = new Pose2d(-44.5,34.5, Math.toRadians(90.0));
    public static Pose2d farSpikeLeftBlue = new Pose2d(-48.5, 39.5, Math.toRadians(80.0));
    public static Pose2d closeSpikeRightBlue = null; //TODO
    public static Pose2d closeSpikeCenterBlue = null; //TODO
    public static Pose2d closeSpikeLeftBlue = null; //TODO
    public static Pose2d apriltagStackBlue = new Pose2d(-57.5, 35, Math.toRadians(180.0));
    public static Pose2d middleStackBlue = new Pose2d(-57.5, 23, Math.toRadians(180.0));
    public static Pose2d wingTrussBlue = new Pose2d(-35, 59, Math.toRadians(180.0));
    public static Pose2d boardTrussBlue = new Pose2d(8, 59, Math.toRadians(180.0));
}
