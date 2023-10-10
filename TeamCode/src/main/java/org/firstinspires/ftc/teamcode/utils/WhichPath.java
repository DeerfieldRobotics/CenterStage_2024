package org.firstinspires.ftc.teamcode.utils;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.utils.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.utils.DrivetrainKotlin;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@Config
//@Autonomous(name = "which path")
public class WhichPath extends OpenCvPipeline {
    private int [] colorCount  = {0,0,0};

    Scalar blu1 = new Scalar(0, 0, 170);
    Scalar blu2 = new Scalar(50, 50, 255);

    int [] red1 = {170, 0, 0};
    int [] red2 = {255, 50, 50};

    private int maxWidth, min, max, left, right, maxY, path = 0;

    Mat workingMatrix = new Mat();
//    Mat lowMat = new Mat();
    Mat redMat = new Mat();

    public int getPath(){ return path;}

    public int [] getDims(){
        return new int[] {workingMatrix.width(), workingMatrix.height()};
    }

    public int [] getCnts(){
        return colorCount;
    }

    @Override
    public Mat processFrame(Mat input) {

        input.copyTo(workingMatrix);

        if(workingMatrix.empty()) {
            return input;
        }
        Imgproc.cvtColor(workingMatrix, workingMatrix, Imgproc.COLOR_RGB2HSV); //

//        Core.inRange(workingMatrix, blu1, blu2, bluMat);
//        Core.inRange(workingMatrix, red1, red2, redMat);

//        Core.add(lowMat, highMat, workingMatrix);

        for(int x  = 0; x < 800; x++){
            for(int y = 0; y < 448; y++){
                double[] c = workingMatrix.get(x,y);

                if(c[0] <= red2[0] && c[0] >= red1[0] && c[1] <= red2[1] && c[1] >= red1[1] && c[2] <= red2[2] && c[2] >= red1[2]) {
                    colorCount[x/(800/3)] ++;
                }
            }
        }

        if(colorCount[0] >= colorCount[1] && colorCount[0] >= colorCount[2]) path = -1;
        else if(colorCount[1] >= colorCount[0] && colorCount[1] >= colorCount[2]) path = 0;
        else path = 1;


        Imgproc.line(input, new Point(800/3,0), new Point(800/3,448), new Scalar(255,0,0), 3);
        Imgproc.line(input, new Point(2*800/3,0), new Point(2*800/3,448), new Scalar(255,0,0), 3);
//        Imgproc.drawMarker(input, new Point(workingMatrix.width()-right, maxY), new Scalar(0,255,0));

        return input;
    }
}
