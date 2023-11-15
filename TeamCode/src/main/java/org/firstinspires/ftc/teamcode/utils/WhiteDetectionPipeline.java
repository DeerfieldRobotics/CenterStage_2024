package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class WhiteDetectionPipeline extends OpenCvPipeline {
    private final int threshold = 220;
    private ArrayList<WhiteFrame> whiteFrames;
    private int startY = 120;
    private int endY = 240;
    private int startX = 0;
    private int endX = 320;
    private int numDivs = 20;
    private int color_counts[] = new int[numDivs];
    double avg = -1;

    public ColorDetectionPipeline() {
        double loc = 0;
    }

    public ColorDetectionPipeline(int numDivs) {
        double loc = 0;
        this.numDivs = numDivs;
    }


    Point leftA = new Point(
            startX,
            startY);
    

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
        for(WhiteFrame frame : whiteFrames){
            sum += frame.getAvgIndex();//get that frame's average index
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

    public StartingPosition getPosition() {
        return position;
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
    }
}