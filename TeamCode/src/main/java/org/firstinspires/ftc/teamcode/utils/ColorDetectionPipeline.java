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