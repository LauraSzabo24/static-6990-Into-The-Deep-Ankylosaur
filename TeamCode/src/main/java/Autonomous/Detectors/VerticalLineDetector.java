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

import TeleOp.SubTests;

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
            new Point(0, 60),
            new Point(320, 240));
    public ArrayList<String> colorList;

    ArrayList<Integer> yellowGoodStreakLengths, yellowStreakPosStart, yellowStreakPosEnd;
    ArrayList<Integer> redGoodStreakLengths, redStreakPosStart, redStreakPosEnd;
    ArrayList<Integer> blueGoodStreakLengths, blueStreakPosStart, blueStreakPosEnd;
    ArrayList<int[]> yellowRanges, blueRanges, redRanges;

    @Override
    public Mat processFrame(Mat input) {
        if(!foundGoodPic)
        {
            Imgproc.rectangle(mat, VerticalLine, new Scalar(255, 0, 0));
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
            Imgproc.rectangle(mat, VerticalLine, new Scalar(0,255,0));

            //region RED MAT
            Mat redMat = mat.submat(VerticalLine);
            Scalar lowHSVRED = new Scalar(-20, 60, 30);
            Scalar highHSVRED = new Scalar(7, 255, 255);
            Core.inRange(redMat, lowHSVRED, highHSVRED, redMat);
            for(int i=0; i<320; i++)
            {
                Rect bar = new Rect(
                        new Point(0, 60),
                        new Point(320, 240));
                double percentRed = ((Core.sumElems(redMat.submat(bar)).val[0]
                        /bar.area())/255);
                if(percentRed>0.3)
                {
                    colorList.add("r");
                }
                else {
                    colorList.add("g");
                }
            }
            //endregion

            //region BLUE MAT
            Mat blueMat = mat.submat(VerticalLine);
            Scalar lowHSVBlue = new Scalar(70, 100, 0);
            Scalar highHSVBlue = new Scalar(110, 255, 200);
            Core.inRange(blueMat, lowHSVBlue, highHSVBlue, blueMat);
            for(int i=0; i<320; i++)
            {
                Rect bar = new Rect(
                        new Point(0, 60),
                        new Point(320, 240));
                double percentBlue = ((Core.sumElems(blueMat.submat(bar)).val[0]
                        /bar.area())/255);
                if(percentBlue>0.3)
                {
                    colorList.add("b");
                }
                else {
                    colorList.add("g");
                }
            }
            //endregion

            //region YELLOW MAT
            Mat yellowMat = mat.submat(VerticalLine);
            Scalar lowHSVYellow = new Scalar(10, 0, 0);
            Scalar highHSVYellow = new Scalar(25, 255, 255);
            Core.inRange(yellowMat, lowHSVYellow, highHSVYellow, yellowMat);
            for(int i=0; i<320; i++)
            {
                Rect bar = new Rect(
                        new Point(0, 60),
                        new Point(320, 240));
                double percentYellow = ((Core.sumElems(yellowMat.submat(bar)).val[0]
                        /bar.area())/255);
                if(percentYellow>0.3)
                {
                    colorList.add("y");
                }
                else {
                    colorList.add("g");
                }
            }
            //endregion

            //region FILL HOLES
            if((!colorList.get(0).equals(colorList.get(23))) && (!colorList.get(0).equals(colorList.get(1))) && (colorList.get(colorList.size()-1).equals(colorList.get(1))))
            {
                colorList.set(0,colorList.get(1));
            }
            if((!colorList.get(colorList.size()-1).equals(colorList.get(colorList.size()-2))) && (!colorList.get(colorList.size()-1).equals(colorList.get(0))) && (colorList.get(colorList.size()-2).equals(colorList.get(0))))
            {
                colorList.set(colorList.size()-1,colorList.get(0));
            }
            for(int i=1; i<colorList.size()-1; i++)
            {
                if((!colorList.get(i).equals(colorList.get(i-1))) && (!colorList.get(i).equals(colorList.get(i+1))) && (colorList.get(i-1).equals(colorList.get(i+1))))
                {
                    colorList.set(i,colorList.get(i+1));
                }
            }
            //endregion

            //region YELLOW FIND RANGES
            int streak = 0;
            yellowGoodStreakLengths = new ArrayList<>();
            yellowStreakPosStart = new ArrayList<>();
            yellowStreakPosEnd = new ArrayList<>();
            for(int i=0; i<colorList.size(); i++)
            {
                if(colorList.get(i).equals("y"))
                {
                    streak++;
                    if(streak==1)
                    {
                        yellowStreakPosStart.add(i);
                    }
                }
                else{
                    if(streak>3)
                    {
                        yellowGoodStreakLengths.add(streak);
                        yellowStreakPosEnd.add(i);
                    }
                    streak = 0;
                }
            }
            if(streak>2)
            {
                yellowGoodStreakLengths.add(streak);
                yellowStreakPosEnd.add(colorList.size()-1);
            }
            //endregion

            //region BLUE FIND RANGES
            streak = 0;
            blueGoodStreakLengths = new ArrayList<>();
            blueStreakPosStart = new ArrayList<>();
            blueStreakPosEnd = new ArrayList<>();
            for(int i=0; i<colorList.size(); i++)
            {
                if(colorList.get(i).equals("b"))
                {
                    streak++;
                    if(streak==1)
                    {
                        blueStreakPosStart.add(i);
                    }
                }
                else{
                    if(streak>3)
                    {
                        blueGoodStreakLengths.add(streak);
                        blueStreakPosEnd.add(i);
                    }
                    streak = 0;
                }
            }
            if(streak>2)
            {
                blueGoodStreakLengths.add(streak);
                blueStreakPosEnd.add(colorList.size()-1);
            }
            //endregion

            //region RED FIND RANGES
            streak = 0;
            redGoodStreakLengths = new ArrayList<>();
            redStreakPosStart = new ArrayList<>();
            redStreakPosEnd = new ArrayList<>();
            for(int i=0; i<colorList.size(); i++)
            {
                if(colorList.get(i).equals("r"))
                {
                    streak++;
                    if(streak==1)
                    {
                        redStreakPosStart.add(i);
                    }
                }
                else{
                    if(streak>3)
                    {
                        redGoodStreakLengths.add(streak);
                        redStreakPosEnd.add(i);
                    }
                    streak = 0;
                }
            }
            if(streak>2)
            {
                redGoodStreakLengths.add(streak);
                redStreakPosEnd.add(colorList.size()-1);
            }
            //endregion
        }
        return mat;
    }
    public ArrayList<int[]> yellowRanges()
    {
        yellowStreakPosStart = new ArrayList<>();
        yellowStreakPosEnd = new ArrayList<>();
        for(int i=0; i< yellowStreakPosStart.size(); i++)
        {
            yellowRanges.add(new int[]{yellowStreakPosStart.get(i), yellowStreakPosEnd.get(i)});
        }
        return yellowRanges();
    }
    public ArrayList<int[]> redRanges()
    {
        redStreakPosStart = new ArrayList<>();
        redStreakPosEnd = new ArrayList<>();
        for(int i=0; i< redStreakPosStart.size(); i++)
        {
            redRanges.add(new int[]{redStreakPosStart.get(i), redStreakPosEnd.get(i)});
        }
        return redRanges();
    }
    public ArrayList<int[]> blueRanges()
    {
        blueStreakPosStart = new ArrayList<>();
        blueStreakPosEnd = new ArrayList<>();
        for(int i=0; i< blueStreakPosStart.size(); i++)
        {
            blueRanges.add(new int[]{blueStreakPosStart.get(i), blueStreakPosEnd.get(i)});
        }
        return blueRanges();
    }
    public void takePicture()
    {
        foundGoodPic = false;
        foundError = false;
        goodDetection = new Mat();
    }
}