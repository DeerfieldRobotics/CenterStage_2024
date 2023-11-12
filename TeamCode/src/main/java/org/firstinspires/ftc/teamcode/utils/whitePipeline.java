package org.firstinspires.ftc.teamcode.utils;


import androidx.annotation.NonNull;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

public class whitePipeline extends OpenCvPipeline
{
    private final int threshold = 25;

    private int whitePixels;

    private Mat hsv = new Mat();
    private Mat v = new Mat();

    private int blue[] = new int[3]; //array with blue pixels, 0 is left, 1 is center, 2 is right
    private int red[] = new int [3]; //array with red pixels, 0 is left, 1 is center, 2 is right
    private int startY = 120;
    private int endY = 240;
    private int startX = 0;
    private int endX = 320;
    private int x1 = 100; //first x division
    private int x2 = 220; //second x division
    private int w1 = 16, w2 = 32, w3 = 48, w4 = 64, w5 = 80, w6 = 96, w7 = 112, w8 = 128, w9 = 144, w10 = 160, w11=176,
            w12=192, w13=208, w14=224, w15=240, w16=256, w17=272, w18=288, w19 = 304, w20 = 320;
//    private String color;
//    private StartingPosition position;

    String whiteVals = "NO WHITES";

    HashMap<Integer, Integer> whites = new HashMap<>();
    TreeMap<Integer, Integer> tm = new TreeMap<>(Collections.reverseOrder());
    double avg = -3;

    double[] selected = new double[4];

    int white[] = new int[20];

//    public ColorDetectionPipeline()
//    {
//        color = "NONE";
//        position = StartingPosition.NONE;
//    }

//    public whitePipeline(String color1)
//    {
//        position = StartingPosition.NONE;
//        color = color1;
//    }

//    public enum Color
//    {
//        RED,
//        BLUE,
//        NONE
//    }


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
    public Mat processFrame(Mat input)
    {
//            avg = -5;
            inputToV(input);
            whiteVals = "";
            for (int i = startY; i < endY; i++) {
                for (int j = startX; j < w6; j++) {
                    if (input.get(i, j)[2] > 90) {
                        white[j % 16]++;
                    }
                }
            }

            for (int i : white) {
                whitePixels = Math.max(whitePixels, i);
            }



            for(int i = 0; i < 20; i++){
                whites.put(white[i],i);
                whiteVals += ("" + white[i]);
            }

            TreeMap<Integer, Integer> tm = new TreeMap<>(Collections.reverseOrder());

            tm.putAll(whites);
//
            avg = ((int) tm.values().toArray()[0] + (int) tm.values().toArray()[1] + (int) tm.values().toArray()[2] + (int) tm.values().toArray()[3])/4.0;

            selected[0] = (int)tm.values().toArray()[0];
            selected[1] = (int)tm.values().toArray()[1];
            selected[2] = (int)tm.values().toArray()[2];
            selected[3] = (int)tm.values().toArray()[3];

//        Imgproc.rectangle(input, leftA, leftB, new Scalar(0, 0, 255), 1);
//        Imgproc.rectangle(input, centerA, centerB, new Scalar(0, 0, 255), 1);
//        Imgproc.rectangle(input, rightA, rightB, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, w1left, w1right, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, w2left, w2right, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, w3left, w3right, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, w4left, w4right, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, w5left, w5right, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, w6left, w6right, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, w7left, w7right, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, w8left, w8right, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, w9left, w9right, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, w10left, w10right, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, w11left, w11right, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, w12left, w12right, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, w13left, w13right, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, w14left, w14right, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, w15left, w15right, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, w16left, w16right, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, w17left, w17right, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, w18left, w18right, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, w19left, w19right, new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, w20left, w20right, new Scalar(0, 0, 255), 1);


        return input;
    }

//    public int[] getWhites(){
////        int[] valz = new int[20];
////        for(int i = 0; i < 20; i++){
////            valz[i] = (int) tm.values().toArray()[i];
////        }
//        return new int[]{(int) tm.values().toArray()[0], (int) tm.values().toArray()[1], (int) tm.values().toArray()[2], (int) tm.values().toArray()[3]};
//    }

    public double getAvg(){
        return avg;
    }

    public String getWhiteVals(){
        return "";
//        return whiteVals;
    }

//    public StartingPosition getPosition()
//    {
//        return position;
//    }


    public double[] getSelected() {
        return selected;
    }

    public int getMaxWhite() { return whitePixels; }

    void inputToV(Mat input)
    {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(hsv, v, 2);
    }

    //    public Color getColor() {
//        return color;
//    }
//    @NonNull
//    public String toString() {
//        return "Position: " + (position==StartingPosition.LEFT ? "LEFT" : (position == StartingPosition.RIGHT ? "RIGHT" : (position == StartingPosition.CENTER ? "CENTER" : "NONE")))
//                + " Color: " + color;
//    }
}