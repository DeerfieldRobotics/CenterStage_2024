package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.TreeSet;

public class WhiteDetectionPipeline extends OpenCvPipeline {
    private final int threshold = 220;
    private ArrayList<WhiteFrame> whiteFrames;
    private final int startY = 120;
    private final int endY = 240;
    private final int startX = 0;
    private final int endX = 320;
    private int numDivs;
    double avg = -1;

    public WhiteDetectionPipeline() {
        numDivs = 20;
    }

    public WhiteDetectionPipeline(int numDivs) {
        this.numDivs = numDivs;
    }

    @Override
    public Mat processFrame(Mat input) {
        WhiteFrame frame = new WhiteFrame(numDivs, 5);
        for (int i = startY; i < endY; i++) {
            for (int j = startX; j < endX; j++) {
                if (input.get(i, j)[0] > threshold && input.get(i, j)[1] > threshold && input.get(i, j)[2] > threshold) {
                    //increment at that index
                    frame.incrementSector(j*numDivs/endX);
                }
            }
        }
        whiteFrames.add(frame);
        //5 averages
        if(whiteFrames.size() > 5) {
            whiteFrames.remove(0);
        }

        //get the average over all frames
        double sum = 0;
        for(WhiteFrame f : whiteFrames){
            sum += f.getAvgIndex();//get that frame's average index
        }
        avg = sum/whiteFrames.size();

        //draw horizontal line
        Imgproc.line(input, new Point(startX, startY), new Point(endX, startY), new Scalar(0, 0, 0), 1);
        //draw lines on screen
        for(int i = 0; i < numDivs-1; i++){
            Imgproc.line(input, new Point(i*endX/numDivs, startY), new Point(i*endX/numDivs, endY), new Scalar(0, 0, 0), 1);
        }

        return input;
    }

    public double getAvg() {
        return avg;
    }

    @NonNull
    public String toString() {
        //return number of pixels in each sector in each frame
        String s = "";
        for(int i = 0; i < whiteFrames.size(); i++){
            s += "Frame " + i + ": ";
            s += whiteFrames.get(i).getAvgIndex();
            // for(WhiteSector sector : frame.getSectors()){
            //     s += sector.getWhiteCount() + " ";
            // }
            s += "\n";
        }

        //add overall average
        s += "Overall Average: " + avg;
        return s;
    }

    static class WhiteFrame {
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

    static class WhiteSector implements Comparable<WhiteSector>{
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

}