package org.firstinspires.ftc.teamcode.utils;

public class Tag {
    public int id;
    public double transX, transY, transZ, yaw, pitch, roll;//feet, degrees

    public Tag(int i, double tx, double ty, double tz, double y, double p, double r){}

    public String toString(){
        String s = "ID: " + id;
        s += "\nTRANSLATIONAL X " + transX;
        s += "\nTRANSLATIONAL Y " + transY;
        s += "\nTRANSLATIONAL Z " + transZ;
        s += "\nYAW " + yaw;
        s += "\nPITCH " + pitch;
        s += "\nROLL " + roll;
        return s;
    }
}
