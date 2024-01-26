package org.firstinspires.ftc.teamcode.utils.detection;

import android.graphics.Canvas;
import android.graphics.Rect;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class ColorDetectionProcessor implements VisionProcessor {

    private final int threshold = 25;
    // private int blue[] = new int[3]; //array with blue pixels, 0 is left, 1 is
    // center, 2 is right
    // private int red[] = new int [3]; //array with red pixels, 0 is left, 1 is
    // center, 2 is right
    private int[] color_counts = new int[3];
    private final ArrayList<Integer> leftCnt = new ArrayList<Integer>();
    private final ArrayList<Integer> centerCnt = new ArrayList<Integer>();
    private final ArrayList<Integer> rightCnt = new ArrayList<Integer>();

    private final int startY = 240;
    private final int endY = 480;
    private final int startX = 0;
    private final int endX = 640;
    private final int x1 = 200; // first x division
    private final int x2 = 440; // second x division
    private String color;
    public static StartingPosition position;

    int targetIndex = 0;
    public enum StartingPosition {
        LEFT,
        CENTER,
        RIGHT,
        NONE
    }
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

    public ColorDetectionProcessor(AllianceHelper.Alliance alliance) {
        //default
        position = StartingPosition.CENTER;
        targetIndex = (alliance == AllianceHelper.Alliance.RED? 0 : 2);
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        color_counts = new int[]{0, 0, 0};

        for (int i = startY; i < endY; i++) {
            for (int j = startX; j < endX; j++) {
                if (frame.get(i, j)[targetIndex] > (frame.get(i, j)[1] + threshold)
                        && frame.get(i, j)[targetIndex] > (frame.get(i, j)[2 - targetIndex] + threshold)) {
                    if (j < x1)
                        color_counts[0]++;
                    else if (j < x2)
                        color_counts[1]++;
                    else
                        color_counts[2]++;
                }
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
        for (int k = 0; k < leftCnt.size(); k++) {
            leftAvg += leftCnt.get(k);
            centerAvg += centerCnt.get(k);
            rightAvg += rightCnt.get(k);
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

        return null;
    }

    public StartingPosition getPosition() { return position; }
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        canvas.drawRect(startX, startY, x1, endY, new android.graphics.Paint());
        canvas.drawRect(x1 + 1, startY, x2, endY, new android.graphics.Paint());
        canvas.drawRect(x2 + 1, startY, endX, endY, new android.graphics.Paint());
    }
}
