package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;

abstract public class PoseHelper {
    public static Pose2d initCloseRed = new Pose2d(10.5, -63, Math.toRadians(90.0));
    public static Pose2d backboardRed = new Pose2d(50, -35, Math.toRadians(180.0));
    public static Pose2d initFarRed = new Pose2d(-39.5, -63, Math.toRadians(90.0));
    public static Pose2d farSpikeRightRed = new Pose2d(-29, -36, Math.toRadians(50.0));
    public static Pose2d farSpikeCenterRed = new Pose2d(-44.5,-34.5, Math.toRadians(90.0));
    public static Pose2d farSpikeLeftRed = new Pose2d(-46, -39.5, Math.toRadians(90.0));
    public static Pose2d apriltagStackRed = new Pose2d(-57.5, -35, Math.toRadians(180.0));
    public static Pose2d middleStackRed = new Pose2d(-57.5, -23, Math.toRadians(180.0));
    public static Pose2d wingTrussRed = new Pose2d(-35, -59, Math.toRadians(180.0));
    public static Pose2d boardTrussRed = new Pose2d(8, -59, Math.toRadians(180.0));
}
