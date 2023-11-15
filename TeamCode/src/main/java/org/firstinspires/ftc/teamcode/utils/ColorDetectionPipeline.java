package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;
import java.util.ArrayList;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorDetectionPipeline extends OpenCvPipeline {
    private final int threshold = 25;
    // private int blue[] = new int[3]; //array with blue pixels, 0 is left, 1 is
    // center, 2 is right
    // private int red[] = new int [3]; //array with red pixels, 0 is left, 1 is
    // center, 2 is right
    private int color_counts[] = new int[3];
    private ArrayList<Integer> leftCnt = new ArrayList<Integer>(), centerCnt = new ArrayList<Integer>(),
            rightCnt = new ArrayList<Integer>();
    private int startY = 120;
    private int endY = 240;
    private int startX = 0;
    private int endX = 320;
    private int x1 = 100; // first x division
    private int x2 = 220; // second x division
    private String color;
    private StartingPosition position;

    int targetIndex = 0;

    public ColorDetectionPipeline() {
        color = "NONE";
        position = StartingPosition.NONE;
    }

    public ColorDetectionPipeline(String color1) {
        //default
        position = StartingPosition.CENTER;
        color = color1;
        targetIndex = (color.equals("RED")? 0 : 2);
    }

    public enum StartingPosition {
        LEFT,
        CENTER,
        RIGHT,
        NONE
    }

    // public enum Color
    // {
    // RED,
    // BLUE,
    // NONE
    // }

    Point leftA = new Point(
            startX,
            startY);
    Point leftB = new Point(
            x1,
            endY);
    Point centerA = new Point(
            x1 + 1,
            startY);
    Point centerB = new Point(
            x2,
            endY);
    Point rightA = new Point(
            x2 + 1,
            startY);
    Point rightB = new Point(
            endX,
            endY);

    //WHITE POInTS

    Point w1left = new Point(
            startX,
            startY);
    Point w1right = new Point(
            w1,
            endY);
    Point w2left = new Point(
            w1+1,
            startY);
    Point w2right = new Point(
            w2,
            endY);
    Point w3left = new Point(
            w2+1,
            startY);
    Point w3right = new Point(
            w3,
            endY);
    Point w4left = new Point(
            w3+1,
            startY);
    Point w4right = new Point(
            w4,
            endY);
    Point w5left = new Point(
            w4+1,
            startY);
    Point w5right = new Point(
            w5,
            endY);
    Point w6left = new Point(
            w5+1,
            startY);
    Point w6right = new Point(
            w6,
            endY);
    Point w7left = new Point(
            w6+1,
            startY);
    Point w7right = new Point(
            w7,
            endY);
    Point w8left = new Point(
            w7+1,
            startY);
    Point w8right = new Point(
            w8,
            endY);
    Point w9left = new Point(
            w8+1,
            startY);
    Point w9right = new Point(
            w9,
            endY);
    Point w10left = new Point(
            w9+1,
            startY);
    Point w10right = new Point(
            w10,
            endY);
    Point w11left = new Point(
            w10+1,
            startY);
    Point w11right = new Point(
            w11,
            endY);
    Point w12left = new Point(
            w11+1,
            startY);
    Point w12right = new Point(
            w12,
            endY);
    Point w13left = new Point(
            w12+1,
            startY);
    Point w13right = new Point(
            w13,
            endY);
    Point w14left = new Point(
            w13+1,
            startY);
    Point w14right = new Point(
            w14,
            endY);
    Point w15left = new Point(
            w14+1,
            startY);
    Point w15right = new Point(
            w15,
            endY);
    Point w16left = new Point(
            w15+1,
            startY);
    Point w16right = new Point(
            w16,
            endY);
    Point w17left = new Point(
            w16+1,
            startY);
    Point w17right = new Point(
            w17,
            endY);
    Point w18left = new Point(
            w17+1,
            startY);
    Point w18right = new Point(
            w18,
            endY);
    Point w19left = new Point(
            w18+1,
            startY);
    Point w19right = new Point(
            w19,
            endY);
    Point w20left = new Point(
            w19+1,
            startY);
    Point w20right = new Point(
            endX,
            endY);


    @Override
    public Mat processFrame(Mat input) {
        color_counts = new int[] { 0, 0, 0 };

        for (int i = startY; i < endY; i++) {
            for (int j = startX; j < endX; j++) {
                if (input.get(i, j)[targetIndex] > (input.get(i, j)[1] + threshold)
                        && input.get(i, j)[targetIndex] > (input.get(i, j)[2-targetIndex] + threshold)) {
                    if (j < x1)
                        color_counts[0]++;
                    else if (j < x2)
                        color_counts[1]++;
                    else
                        color_counts[2]++;
                
                }
                // } else if (color.equals("BLUE") && (input.get(i, j)[2] > (input.get(i, j)[1] + threshold)
                //         && input.get(i, j)[2] > (input.get(i, j)[0] + threshold))) {
                //     if (j < x1)
                //         color_counts[0]++;
                //     else if (j < x2)
                //         color_counts[1]++;
                //     else
                //         color_counts[2]++;
                // }
            }

        leftCnt.add(color_counts[0]);
        centerCnt.add(color_counts[1]);
        rightCnt.add(color_counts[2]);

        if (leftCnt.size() > 5) {
            leftCnt.remove(0);
            centerCnt.remove(0);
            rightCnt.remove(0);
        }

        // take the average of each arraylist and compare them to determine position
        int leftAvg = 0, centerAvg = 0, rightAvg = 0;
        for (int i = 0; i < leftCnt.size(); i++) {
            leftAvg += leftCnt.get(i);
            centerAvg += centerCnt.get(i);
            rightAvg += rightCnt.get(i);
        }

        leftAvg /= leftCnt.size();
        centerAvg /= centerCnt.size();
        rightAvg /= rightCnt.size();

        if (leftAvg > centerAvg && leftAvg > rightAvg)
            position = StartingPosition.LEFT;
        else if (centerAvg > leftAvg && centerAvg > rightAvg)
            position = StartingPosition.CENTER;
        else if (rightAvg > leftAvg && rightAvg > centerAvg)
            position = StartingPosition.RIGHT;
        else
            position = StartingPosition.CENTER;

        // int maxRed = Math.max(red[0], Math.max(red[1], red[2]));
        // int maxBlue = Math.max(blue[0], Math.max(blue[1], blue[2]));

        // if (maxRed>maxBlue) {
        // if(maxRed == red[0])
        // position = StartingPosition.LEFT;
        // else if(maxRed == red[1])
        // position = StartingPosition.CENTER;
        // else
        // position = StartingPosition.RIGHT;
        // }
        // else {
        // if(maxBlue == blue[0])
        // position = StartingPosition.LEFT;
        // else if(maxBlue == blue[1])
        // position = StartingPosition.CENTER;
        // else
        // position = StartingPosition.RIGHT;
        // }

        Imgproc.rectangle(input, leftA, leftB, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, centerA, centerB, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, rightA, rightB, new Scalar(0, 0, 255), 1);


        return input;
    }

    public StartingPosition getPosition() {
        return position;
    }

    // public Color getColor() {
    // return color;
    // }
    @NonNull
    public String toString() {
        return "Position: "
                + (position == StartingPosition.LEFT ? "LEFT"
                        : (position == StartingPosition.RIGHT ? "RIGHT"
                                : (position == StartingPosition.CENTER ? "CENTER" : "NONE")))
                + " Color: " + color;
    }
}