package org.firstinspires.ftc.teamcode.utils.detection;

import org.firstinspires.ftc.teamcode.utils.hardware.Drivetrain;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.TreeSet;

public class WhiteDetectionPipeline2 extends OpenCvPipeline {
    private final int threshold = 240;
    private final ArrayList<WhiteFrame> whiteFrames;
    private final int startY = 120;
    private final int endY = 240;
    private final int startX = 0;
    private final int endX = 200;
    private final int numSectors;
    double position = -2;
    private static String selectedSectors;

    public WhiteDetectionPipeline2(Drivetrain drivetrain) {
        numSectors = 10;
        whiteFrames = new ArrayList<>();
        selectedSectors = "";
    }

    public WhiteDetectionPipeline2(int numSectors) {
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
                    frame.incrementXValue(j* numSectors /endX, j);
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
            if(!f.whiteSectors.isEmpty())
                sum += f.getLocation();//get that frame's average index

        }
        position = sum/whiteFrames.size();

        // position line
        Imgproc.line(input, new Point(position, startY), new Point(position, endY), new Scalar(255, 0, 0), 1);

        //draw horizontal line
        Imgproc.line(input, new Point(startX, startY), new Point(endX, startY), new Scalar(0, 0, 0), 1);
        //draw lines on screen
        for(int i = 0; i < numSectors -1; i++){
            Imgproc.line(input, new Point((double)i*endX/ numSectors, startY), new Point((double)i*endX/ numSectors, endY), (!whiteFrames.isEmpty() && !whiteFrames.get(0).validSectors.isEmpty() && ((whiteFrames.get(0).validSectors.contains(i)) || (i != 0 && whiteFrames.get(0).validSectors.contains(i-1)))) ? new Scalar(0, 255, 0) : new Scalar(0, 0, 0), 1);
        }

        return input;
    }

    public double getPosition() {
        return position;
    }

    static class WhiteFrame {
        int maxSize;
        //number of sectors to consider
        int numSectors; //pixel width in sectors
        private final ArrayList<WhiteSector> whiteSectors;
        private final ArrayList<Integer> validSectors = new ArrayList<>();
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
        public void incrementXValue(int index, int x) {
            whiteSectors.get(index).addXValue(x);
        }
        public int getLocation() {
            ArrayList<Integer> xValues = new ArrayList<>();
            TreeSet<WhiteSector> sortedSectors = new TreeSet<>(whiteSectors);
            validSectors.clear();
            for(int i = 0; i<Math.min(numSectors,sortedSectors.size()); i++) {
                WhiteSector ws = sortedSectors.pollFirst();
                xValues.addAll(ws.xValues);
                validSectors.add(ws.getIndex());
            }
            Collections.sort(xValues);
            if(!xValues.isEmpty())
                return xValues.get(xValues.size()/2);
            else return -1;
        }
    }

    static class WhiteSector implements Comparable<WhiteSector>{
        private int whiteCount;
        private final int index;
        public ArrayList<Integer> xValues = new ArrayList<>();
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
        public void addXValue(int x) { xValues.add(x); }
        public int getIndex() {
            return index;
        }
        @Override
        public int compareTo(WhiteSector whiteSector) {
            return -whiteCount + whiteSector.getWhiteCount();
        }
    }
}