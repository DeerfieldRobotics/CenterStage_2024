package org.firstinspires.ftc.teamcode.utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorDetectionPipelineWorking extends OpenCvPipeline {


    Scalar low1 = new Scalar(0, 150, 150);
    Scalar low2 = new Scalar(5, 255, 255);
    Scalar high1 = new Scalar(245, 150, 150);
    Scalar high2 = new Scalar(255, 255, 255);


    private int maxWidth, min, max, left, right, maxY;

    Mat workingMatrix = new Mat();
    Mat lowMat = new Mat();
    Mat highMat = new Mat();

    @Override
    public Mat processFrame(Mat input) {

        input.copyTo(workingMatrix);

        if(workingMatrix.empty()) {
            return input;
        }
        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2HSV); //

        Core.inRange(workingMatrix, low1, low2, lowMat);
        Core.inRange(workingMatrix, high1, high2, highMat);

        Core.add(lowMat, highMat, workingMatrix);

        maxWidth = 0;
        for(int y = 0; y < workingMatrix.height();y++) {
            min = workingMatrix.width();
            max = 0;
            int l = workingMatrix.width();
            int r = 0;
            for(int x = 0; x<workingMatrix.width() && (min == workingMatrix.width() || max == 0);x++){

                if(min == workingMatrix.width()&&workingMatrix.get(y,x)[0]!=0) {
                    min = x;
                    l = x;
                }
                if(max == 0&&workingMatrix.get(y,workingMatrix.width()-1-x)[0]!=0) {
                    max = workingMatrix.width()-1-x;
                    r = x;
                }
            }

            if(max-min>maxWidth) {
                maxWidth = max-min;
                left = l;
                right = r;
                maxY = y;
            }
        }


        Imgproc.drawMarker(input, new Point(left, maxY), new Scalar(0,255,0));
        Imgproc.drawMarker(input, new Point(workingMatrix.width()-right, maxY), new Scalar(0,255,0));

        return input;
    }


    public int height() {
        return workingMatrix.height();
    }

    public int width() {
        return workingMatrix.width();
    }

    public int getMaxWidth() { //use this to align with cone, when maxwidth reaches certain threshold after centered to reasonable threshold of accuracy based on the difference of the average of LDist1 and RDist 2 and the average of LDist2 and RDist 1
        return maxWidth;
    }

    public int getLeft() {
        return left;
    }

    public int getRight() {
        return right;
    }

}