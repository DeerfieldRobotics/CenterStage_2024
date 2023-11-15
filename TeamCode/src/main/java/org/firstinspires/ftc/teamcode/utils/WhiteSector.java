package org.firstinspires.ftc.teamcode.utils;

public class WhiteSector implements Comparable<WhiteSector>{
    private int whiteCount;
    private final int index;

    //default constructor
    public WhiteSector() {
        whiteCount = 0;
        index = 0;
    }

    public WhiteSector(int whiteCount, int index) {
        this.whiteCount = whiteCount;
        this.index = index;
    }

    public int getWhiteCount() {
        return whiteCount;
    }

    public void incrementWhiteCount() {
        whiteCount++;
    }

    public int getIndex() {
        return index;
    }

    @Override
    public int compareTo(WhiteSector whiteSector) {
        return -whiteCount + whiteSector.getWhiteCount();
    }
}
