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
    private final ArrayList<WhiteFrame> whiteFrames;
    private final int startY = 120;
    private final int endY = 240;
    private final int startX = 0;
    private final int endX = 200;
    private final int numSectors;
    double avg = -2;
    private static String selectedSectors;

    public WhiteDetectionPipeline() {
        numSectors = 10;
        whiteFrames = new ArrayList<>();
        selectedSectors = "";
    }

    public WhiteDetectionPipeline(int numSectors) {
        whiteFrames= new ArrayList<>();
        this.numSectors = numSectors;
        selectedSectors = "";
    }

    @Override
    public Mat processFrame(Mat input) {
        WhiteFrame frame = new WhiteFrame(numSectors, 3); //3 sector wide pixel image

        for(int i = 0; i < numSectors; i++) frame.addSector(new WhiteSector(0,i));

        //loop through each pixel in the frame and puts it in the correct sector
        for (int i = startY; i < endY; i++) {
            for (int j = startX; j < endX; j++) {
                if (input.get(i, j)[0] > threshold && input.get(i, j)[1] > threshold && input.get(i, j)[2] > threshold) {
                    //increment at that index
                    frame.incrementSector(j* numSectors /endX);
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
        for(int i = 0; i < numSectors -1; i++){
            Imgproc.line(input, new Point((double)i*endX/ numSectors, startY), new Point((double)i*endX/ numSectors, endY), new Scalar(0, 0, 0), 1);
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
        StringBuilder s = new StringBuilder();
        for(int i = 0; i < whiteFrames.size(); i++){
            s.append("Frame ").append(i).append(": ");
            s.append(whiteFrames.get(i).toString());
            // for(WhiteSector sector : frame.getSectors()){
            //     s += sector.getWhiteCount() + " ";
            // }
//            s += "\n";
        }
//
//        //add overall average
//        String s = "Overall Average: " + avg;
        return s.toString();
    }

    static class WhiteFrame {
        int maxSize;
        //number of sectors to consider
        int numSectors; //pixel width in sectors
        private final ArrayList<WhiteSector> whiteSectors;

        public WhiteFrame(int maxSize, int numSectors) {
            this.numSectors = numSectors;
            whiteSectors = new ArrayList<>();
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
            StringBuilder selectedSectorsStringBuilder = new StringBuilder();
            for(int i = 0; i < Math.min(numSectors,sortedSectors.size()); i++){
                WhiteSector ws = sortedSectors.pollFirst();
                sum += ws.getIndex();
                selectedSectorsStringBuilder.append(ws.getIndex()).append(" ");
            }

            selectedSectors = selectedSectorsStringBuilder.append("\n").toString();

            sortedSectors.clear();

            return ((double) sum)/numSectors;
        }

        @NonNull
        public String toString(){
            StringBuilder s = new StringBuilder();
            for(WhiteSector se : whiteSectors) s.append(se.getWhiteCount()).append(" ");

            s.append("\n").append(selectedSectors).append("\n");

            return s.toString();
        }
    }

    static class WhiteSector implements Comparable<WhiteSector>{
        private int whiteCount;
        private final int index;

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