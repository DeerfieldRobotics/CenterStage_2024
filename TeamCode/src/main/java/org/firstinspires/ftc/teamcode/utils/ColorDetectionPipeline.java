package org.firstinspires.ftc.teamcode.utils;


import androidx.annotation.NonNull;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorDetectionPipeline extends OpenCvPipeline
{
    private final int threshold = 25;
    private int blue[] = new int[3]; //array with blue pixels, 0 is left, 1 is center, 2 is right
    private int red[] = new int [3]; //array with red pixels, 0 is left, 1 is center, 2 is right
    private int startY = 120;
    private int endY = 240;
    private int startX = 0;
    private int endX = 320;
    private int x1 = 100; //first x division
    private int x2 = 220; //second x division

    public enum StartingPosition
    {
        LEFT,
        CENTER,
        RIGHT,
        NONE
    }

    public enum Color
    {
        RED,
        BLUE,
        NONE
    }

    private StartingPosition position = StartingPosition.NONE;
    private Color color = Color.NONE;
    Point leftA = new Point(
            startX,
            startY);
    Point leftB = new Point(
            x1,
            endY);
    Point centerA = new Point(
            x1+1,
            startY);
    Point centerB = new Point(
            x2,
            endY);
    Point rightA = new Point(
            x2+1,
            startY);
    Point rightB = new Point(
            endX,
            endY);

    @Override
    public Mat processFrame(Mat input)
    {
        for(int i = startY; i < endY; i++) {
            for(int j = startX; j < endX; j++) {
                if(input.get(i, j)[0] > (input.get(i, j)[1] + threshold) && input.get(i, j)[0] > (input.get(i, j)[2] + threshold)) {
                    if(j < x1)
                        red[0]++;
                    else if(j < x2)
                        red[1]++;
                    else
                        red[2]++;
                }
                else if(input.get(i,j)[2] > (input.get(i,j)[1]+threshold) && input.get(i,j)[2] > (input.get(i,j)[0]+threshold)) {
                    if(j < x1)
                        blue[0]++;
                    else if(j < x2)
                        blue[1]++;
                    else
                        blue[2]++;
                }
            }
        }

        int maxRed = Math.max(red[0], Math.max(red[1], red[2]));
        int maxBlue = Math.max(blue[0], Math.max(blue[1], blue[2]));

        if (maxRed>maxBlue) {
            color = Color.RED;
            if(maxRed == red[0])
                position = StartingPosition.LEFT;
            else if(maxRed == red[1])
                position = StartingPosition.CENTER;
            else
                position = StartingPosition.RIGHT;
        }
        else {
            color = Color.BLUE;
            if(maxBlue == blue[0])
                position = StartingPosition.LEFT;
            else if(maxBlue == blue[1])
                position = StartingPosition.CENTER;
            else
                position = StartingPosition.RIGHT;
        }

        Imgproc.rectangle(input, leftA, leftB, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, centerA, centerB, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, rightA, rightB, new Scalar(0, 0, 255), 1);

        return input;
    }

    public StartingPosition getPosition()
    {
        return position;
    }
    public Color getColor() {
        return color;
    }
    @NonNull
    public String toString() {
        return "Position: " + (position==StartingPosition.LEFT ? "LEFT" : (position == StartingPosition.RIGHT ? "RIGHT" : (position == StartingPosition.CENTER ? "CENTER" : "NONE")))
                + " Color: " + (color==Color.RED ? "RED" : (color == Color.BLUE ? "BLUE" : "NONE"));
    }
}