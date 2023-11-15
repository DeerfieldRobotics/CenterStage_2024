package org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayList;
import java.util.TreeSet;

public class WhiteFrame {
    int maxSize = 20;
    //number of sectors to consider
    int numSectors = 5;
    private ArrayList<WhiteSector> whiteSectors;
    double avg = -1;

    public WhiteFrame(int maxSize, int numSectors) {
        whiteSectors = new ArrayList<WhiteSector>();
        this.maxSize = maxSize;
    }

    public void addSector(WhiteSector sector) {
        whiteSectors.add(sector);
        if(whiteSectors.size() > maxSize) {
            whiteSectors.remove(0);
        }
    }

    public void incrementSector(int index) {
        whiteSectors.get(index).incrementWhiteCount();
    }

    public double getAvgIndex(){

        TreeSet<WhiteSector> sortedSectors = new TreeSet<>(whiteSectors);

        //get the average index of the first numSectors sectors
        int sum = 0;   
        for(int i = 0; i < numSectors; i++){
            sum += sortedSectors.pollFirst().getIndex();
        }

        sortedSectors.clear();

        return ((double) sum)/numSectors;
    }
}
