package org.firstinspires.ftc.teamcode.utils;

public class Tag {
    public int id;
    public double transX, transY, transZ, yaw, pitch, roll;//feet, degrees

    public Tag(int i, double tx, double ty, double tz, double y, double p, double r){}

    public String toString(){
        return "ID: " + id
        + "\nTRANSLATIONAL X " + transX
        + "\nTRANSLATIONAL Y " + transY
        + "\nTRANSLATIONAL Z " + transZ
        + "\nYAW " + yaw
        + "\nPITCH " + pitch
        + "\nROLL " + roll;
    }
}
