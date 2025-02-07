package Autonomous.Detectors;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class VerticalLineDetector extends OpenCvPipeline {
    //region RANDOM JUNK
    Telemetry telemetry;
    Mat mat = new Mat();
    Mat goodDetection;
    public VerticalLineDetector(Telemetry t) { telemetry = t; }
    boolean foundError = false;
    boolean foundGoodPic = true;
    //endregion

    //region FOR REFERENCE (0,0) in top left
        /*static final Rect SCREENSIZEBOX = new Rect( //make this the correct area
            new Point(0, 60),
            new Point(320, 240));
        Scalar lowHSVYELLOW= new Scalar(10, 0, 0);
        Scalar highHSVYELLOW = new Scalar(25, 255, 255);

        Scalar lowHSVBLUE = new Scalar(70, 100, 0);
        Scalar highHSVBLUE = new Scalar(110, 255, 200);

        Scalar lowHSVREDD = new Scalar(10, 0, 0);
        Scalar highHSVREDD = new Scalar(175, 255, 255);

        Scalar lowHSVGREEN= new Scalar(45, 0, 0);
        Scalar highHSVGREEN = new Scalar(76, 255, 255);

        Scalar lowHSVPURPLE= new Scalar(120, 50, 50);
        Scalar highHSVPURPLE = new Scalar(170, 255, 255);

        Scalar lowHSVWHITE= new Scalar(0, 0, 80);
        Scalar highHSVWHITE = new Scalar(180, 30, 255);*/
    //endregion
    static final Rect VerticalLine = new Rect(
            new Point(100, 60),
            new Point(170, 120));
    @Override
    public Mat processFrame(Mat input) {
        if(!foundGoodPic)
        {
            //region GET COLORS IN RANGE
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
            Imgproc.rectangle(mat, VerticalLine, new Scalar(0,255,0));


            Mat redMat = mat.submat(VerticalLine);
            Scalar lowHSVRED = new Scalar(-20, 60, 30);
            Scalar highHSVRED = new Scalar(7, 255, 255);
            Core.inRange(redMat, lowHSVRED, highHSVRED, redMat);

            //do for yellow and blue
            //combine mats to create a pic that has black, red, yellow, blue
            //average out the lines in the strip ->
            //if 2/10 is yellow yellow wins
            //if less than 2 lines worth of pixel change it to neighboring color
            //record colors and amount of lines each has in order
            //check if good block close enough if not scan next range
            //turn to distance
            //return distance, stop filming
        }
        return mat;
    }
    public boolean doneFilming(){
        return foundGoodPic;
    }
    public void takePicture()
    {
        foundGoodPic = false;
        foundError = false;
        goodDetection = new Mat();
    }
}