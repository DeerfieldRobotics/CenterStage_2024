package org.firstinspires.ftc.teamcode.utils;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvPipeline;
public class ColorDetectionPipeline extends OpenCvPipeline{
    private int thresholdRed = 25;
    private int thresholdBlue = 25;
    //points bounding the regions of the screen that define the left, center, and right sections
    Point regionLeftPointA = new Point(
            1,
            0);
    Point regionLeftPointB = new Point(
            320,
            480);
    Point regionCenterPointA = new Point(
            160,
            0);
    Point regionCenterPointB = new Point(
            480,
            480);
    Point regionRightPointA = new Point(
            320,
            0);
    Point regionRightPointB = new Point(
            640,
            480);

    Mat regionLeft, regionCenter, regionRight = new Mat();
    String propPosition = "";
    int totalLeftRed, totalRightRed, totalCenterRed, totalLeftBlue, totalRightBlue, totalCenterBlue;
    int[][] totalRedBlue = {{totalLeftRed, totalCenterRed, totalRightRed}, {totalLeftBlue, totalCenterBlue, totalRightBlue}};
    @Override
    public Mat processFrame(Mat input) {
        //number of threshold color pixels in left, center, and right portions of the screen
        int countLeftRed = 0, countCenterRed = 0, countRightRed = 0, countLeftBlue = 0, countCenterBlue = 0, countRightBlue = 0;
        //checks color of each pixel of the matrix (image)
        for(int i = 240; i < 480; i++) {
            for (int j = 0; j < 640; j++) {
                if (input.get(i, j)[0] > (input.get(i, j)[1] + thresholdRed) && input.get(i, j)[0] > (input.get(i, j)[2] + thresholdRed)) {
                    if (j < 213) {
                        countLeftRed++;
                    } else if (j < 427) {
                        countCenterRed++;
                    } else {
                        countRightRed++;
                    }
                }
                if (input.get(i, j)[2] > (input.get(i, j)[1] + thresholdBlue) && input.get(i, j)[2] > (input.get(i, j)[0] + thresholdBlue)) {
                    if (j < 213) {
                        countLeftBlue++;
                    } else if (j < 427) {
                        countCenterBlue++;
                    } else {
                        countRightBlue++;
                    }
                }
            }
        }
        int maxOneTwoRed = Math.max(countRightRed, countCenterRed);
        int maxRed = Math.max(maxOneTwoRed, countLeftRed);
        int maxOneTwoBlue = Math.max(countRightRed, countCenterRed);
        int maxBlue = Math.max(maxOneTwoBlue, countLeftRed);
        //add something here this is unfinished
        if(maxRed == countLeftRed || maxBlue == countLeftBlue)
        {
            propPosition = "left";
        } else if(maxRed == countCenterRed || maxBlue == countCenterBlue)
        {
            propPosition = "center";
        } else
        {
            propPosition = "right";
        }

        return input;
    }
    public int[][] getTotalRedBlue(){
        return totalRedBlue;
    }
    public String getPropPosition(){
        return propPosition;
    }

}
