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
    private final int threshold = 240;
    private ArrayList<WhiteFrame> whiteFrames;
    private final int startY = 120;
    private final int endY = 240;
    private final int startX = 0;
    private final int endX = 200;
    private int numDivs;
    double avg = -2;
    private static String selectedSectors;

    public WhiteDetectionPipeline() {
        numDivs = 10;
        whiteFrames = new ArrayList<>();
        selectedSectors = "";
    }

    public WhiteDetectionPipeline(int numDivs) {
        whiteFrames= new ArrayList<>();
        this.numDivs = numDivs;
        selectedSectors = "";
    }

    @Override
    public Mat processFrame(Mat input) {
        WhiteFrame frame = new WhiteFrame(numDivs, 3);

        for(int i = 0; i < numDivs; i++) frame.addSector(new WhiteSector(0,i));

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
        while(whiteFrames.size() > 5) {
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
        if(whiteFrames.size()==0) return "";
        //return number of pixels in each sector in each frame
        String s = "";
        for(int i = 0; i < whiteFrames.size(); i++){
            s += "Frame " + i + ": ";
            s += whiteFrames.get(i).toString();
            // for(WhiteSector sector : frame.getSectors()){
            //     s += sector.getWhiteCount() + " ";
            // }
//            s += "\n";
        }
//
//        //add overall average
//        String s = "Overall Average: " + avg;
        return s;
    }

    static class WhiteFrame {
        int maxSize = 20;
        //number of sectors to consider
        int numSectors;
        private ArrayList<WhiteSector> whiteSectors;
        double avg = -1;

        public WhiteFrame(int maxSize, int numSectors) {
            this.numSectors = numSectors;
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
            selectedSectors = "";
            TreeSet<WhiteSector> sortedSectors = new TreeSet<>(whiteSectors);

            //get the average index of the first numSectors sectors
            int sum = 0;
            for(int i = 0; i < Math.min(3,sortedSectors.size()); i++){
                WhiteSector ws = sortedSectors.pollFirst();
                sum += ws.getIndex();
                selectedSectors += ws.getIndex() + " ";
            }

            selectedSectors+= "\n";

            sortedSectors.clear();

            return ((double) sum)/numSectors;
        }

        public String toString(){
            String s = "";
            for(WhiteSector se : whiteSectors) s += ((se.getWhiteCount()) + " ");

            s +="\n"+selectedSectors + "\n";

            return s;
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