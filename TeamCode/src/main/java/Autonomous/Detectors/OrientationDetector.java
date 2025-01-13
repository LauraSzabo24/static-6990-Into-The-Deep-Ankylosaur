package Autonomous.Detectors;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
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

public class OrientationDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    Mat goodDetection;
    public OrientationDetector(Telemetry t) { telemetry = t; }

    //region FOR REFERENCE
        /*Scalar lowHSVYELLOW= new Scalar(10, 0, 0);
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
    //region for reference only (0,0) in top left
    static final Rect SCREENSIZEBOX = new Rect( //make this the correct area
            new Point(0, 60),
            new Point(320, 240));
    //endregion
    //region roi actual boxes
    static final Rect LEFT_ROI = new Rect( //make this the correct area
            new Point(100, 60),
            new Point(170, 120));
    static final Rect CENTER_ROI = new Rect( //make this the correct area
            new Point(230, 70),
            new Point(292, 170));
    //endregion
    pointVector xLine, yLine, zLine;
    double centerLineOffset = 0;
    double recordArea;
    String printable;
    boolean foundError = false;
    boolean foundGoodPic = true;
    @Override
    public Mat processFrame(Mat input) {
        if(!foundGoodPic)
        {
            //region GET COLORS
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

            Scalar lowHSVRED = new Scalar(-20, 60, 30);
            Scalar highHSVRED = new Scalar(7, 255, 255);
            Core.inRange(mat, lowHSVRED, highHSVRED, mat);
            //endregion

            //region GET CONTOURS
            Mat canMat = new Mat();
            Imgproc.GaussianBlur(mat, canMat, new Size(5.0, 15.0), 0.00);
            Imgproc.Canny(canMat, canMat, 10, 10); //100,200

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));  // Define a kernel

            Mat erodedOutput = new Mat();
            Imgproc.erode(canMat, erodedOutput, kernel);

            Mat dilatedOutput = new Mat();
            Imgproc.dilate(canMat, dilatedOutput, kernel);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(canMat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            //endregion

            //region FILTER OUT EXTERNAL CONTOURS
            MatOfPoint recordMat = new MatOfPoint();
            recordArea = 600;
            for (MatOfPoint cont : contours) {
                if (Imgproc.contourArea(cont) > recordArea) {
                    recordArea = Imgproc.contourArea(cont);
                    recordMat = cont;
                }
            }

            List<MatOfPoint> recordMatList = new ArrayList<>();
            if (recordMat.empty()) {
                foundError = true;
            }
            //endregion
            else {
                recordMatList.add(recordMat);
                Mat contourImage = new Mat(canMat.size(), CvType.CV_8UC3, new Scalar(0, 0, 0)); // Create a black image
                Imgproc.drawContours(contourImage, recordMatList, -1, new Scalar(255, 255, 255), 1);

                if (!foundError && !contours.isEmpty() && recordMatList.get(0).toList().size() > 10) {
                    //region IMPORTANT POINTS
                    Point lowest = new Point();
                    double lowRecord = 0;
                    Point highest = new Point();
                    double highRecord = Integer.MAX_VALUE;

                    int startCol = 0;
                    int startRow = 0;
                    int rowCounter = 0;
                    Point highStart = new Point();
                    Point lowStart = new Point();

                    int endCol = 0;
                    Point highEnd = new Point();
                    Point lowEnd = new Point();

                    Point startHighAnchor = new Point();
                    Point startLowAnchor = new Point();
                    //endregion

                    //region GET GENERAL HIGHEST AND LOWEST POINTS
                    List<Point> pts = new ArrayList<>();
                    List<Point> lowPoints = new ArrayList<>();
                    List<Point> highPoints = new ArrayList<>();
                    for (int c = 0; c < contourImage.cols(); c++) {
                        for (int r = 0; r < contourImage.rows(); r++) {
                            if (contourImage.get(r, c)[0] == 255 && contourImage.get(r, c)[1] == 255 && contourImage.get(r, c)[2] == 255) {
                                //region START COLUMN SHENANIGANS
                                if (startCol == 0) {
                                    startCol = c;
                                    startRow = r;
                                } else if (Math.abs(startCol - c) == 3 && rowCounter == 0) {
                                    startHighAnchor = new Point(c, r);
                                    rowCounter++;
                                } else if (Math.abs(startCol - c) == 3 && rowCounter == 1) {
                                    startLowAnchor = new Point(c, r);
                                    rowCounter++;
                                }
                                //endregion

                                if (r > lowRecord) {
                                    lowRecord = r;
                                    lowest = new Point(c, r);
                                    if (Math.abs(startCol - c) < 20 && Math.abs(startCol - c) > 6) {
                                        lowPoints.add(lowest);
                                    }
                                }
                                if (r < highRecord) {
                                    highRecord = r;
                                    highest = new Point(c, r);
                                    if (Math.abs(startCol - c) < 15) {
                                        highPoints.add(highest);
                                    }
                                }
                            }
                        }
                    }
                    //endregion

                    //region FIND LOWEST AND HIGHEST POINTS
                    double recordAngl = -1;
                    if (lowPoints.isEmpty()) {
                        lowStart = new Point(startCol, startRow);
                    } else {
                        lowStart = lowPoints.get(0);
                    }

                    for (int i = 0; i < lowPoints.size() - 3; i++) {
                        double currAngl = Math.abs(angleBtwn(lowPoints.get(i), lowPoints.get(i + 3)));
                        if (currAngl > recordAngl) {
                            lowStart = lowPoints.get(i);
                            recordAngl = currAngl;
                        }
                    }

                    highStart = new Point(startCol, startRow);
                    //double recordAnglDiff = Math.abs(angleBtwn(highStart, startLowAnchor) - angleBtwn(highStart, startHighAnchor));
                    double recordAnglDiff = 30; //otherwise always choose high start
                    for (int i = 3; i < highPoints.size() - 3; i++) {
                        double currForAngl = Math.abs(angleBtwn(highPoints.get(i), highPoints.get(i + 3)));
                        double currBackAngle = Math.abs(angleBtwn(highPoints.get(i), highPoints.get(i - 3)));
                        double currAnglDiff = Math.abs(currBackAngle - currForAngl);
                        if (currAnglDiff > recordAnglDiff) {
                            highStart = highPoints.get(i);
                            recordAnglDiff = currAnglDiff;
                        }
                    }
                    //endregion

                    //region GET END POINTS
                    lowRecord = 0;
                    highRecord = Integer.MAX_VALUE;
                    boolean found = false;
                    for (int c = contourImage.cols() - 1; c > 0; c--) {
                        for (int r = contourImage.rows() - 1; r > 0; r--) {
                            if (contourImage.get(r, c)[0] == 255 && contourImage.get(r, c)[1] == 255 && contourImage.get(r, c)[2] == 255) {
                                if (endCol == 0) {
                                    endCol = c;
                                }

                                if (r > lowRecord) {
                                    lowRecord = r;
                                    if (Math.abs(endCol - c) < 15) {
                                        lowEnd = new Point(c, r);
                                        found = true;
                                    }
                                }
                                if (r < highRecord) {
                                    highRecord = r;
                                    if (Math.abs(endCol - c) < 3) {
                                        highEnd = new Point(c, r);
                                    }
                                }
                            }
                        }
                    }
                    //endregion

                    //region CHECK SIDES AND REMEASURE
                    double xside = distance(lowStart, lowest);
                    double yside = distance(highStart, highest);
                    double zside = distance(highStart, lowStart);
                    double aside = distance(lowest, lowEnd);
                    double bside = distance(highest, highEnd);
                    double cside = distance(highEnd, lowEnd);

                    //check if need new picture
                    double yadiff = Math.abs(yside - aside);
                    printable = "BADDDDD yside = " + yside + " aside = " + aside + " difference = " + Math.abs(yside - aside);
                    if (yadiff > 0 && yadiff < 100) { //works except purple star in bad spot when short side
                        printable = "GOOOOD yside = " + yside + " aside = " + aside + " difference = " + Math.abs(yside - aside);
                    } else {
                        foundError = true;
                    }

                    //see if zx or cb is better
                    if (!foundError) {
                        double constantZRatio = 10;
                        double zxRatioOffset = Math.abs((zside / xside) - constantZRatio);
                        double cbRatioOffset = Math.abs((cside / bside) - constantZRatio);

                        printable = "BADDD zxratiooffset " + zxRatioOffset + " cbratiooffset = " + cbRatioOffset;
                        if (zxRatioOffset < cbRatioOffset && zxRatioOffset > 5) // use z and x
                        {
                            printable = "USE ZX zxratiooffset " + zxRatioOffset + " cbratiooffset = " + cbRatioOffset;
                            foundGoodPic = true;
                            xLine = new pointVector(lowStart, lowest, xside);
                            yLine = new pointVector(highStart, highest, yside);
                            zLine = new pointVector(highStart, lowStart, zside);
                        } else if (cbRatioOffset > 5)  //use c and b
                        {
                            foundGoodPic = true;
                            printable = "USE CB zxratiooffset " + zxRatioOffset + " cbratiooffset = " + cbRatioOffset;
                            xLine = new pointVector(highest, highEnd, bside);
                            yLine = new pointVector(highEnd, lowEnd, cside);
                            zLine = new pointVector(lowest, lowEnd, aside);
                        }
                    }
                    //endregion

                    //region DRAW VECTORS
                    if (!foundError) {
                        pts.add(lowest);
                        pts.add(highest);
                        pts.add(highStart);
                        pts.add(lowStart);
                        pts.add(highEnd);
                        pts.add(lowEnd);
                        Imgproc.drawMarker(contourImage, pts.get(0), new Scalar((int) (255), (int) (0), (int) (0))); //lowest-red
                        Imgproc.drawMarker(contourImage, pts.get(1), new Scalar((int) (255), (int) (100), (int) (100))); //highest-brown
                        Imgproc.drawMarker(contourImage, pts.get(2), new Scalar((int) (0), (int) (255), (int) (0)));  //highStart-green
                        Imgproc.drawMarker(contourImage, pts.get(3), new Scalar((int) (255), (int) (255), (int) (0)));  //lowStart-yellow
                        Imgproc.drawMarker(contourImage, pts.get(4), new Scalar((int) (0), (int) (0), (int) (255)));  //highEnd-blue
                        Imgproc.drawMarker(contourImage, pts.get(5), new Scalar((int) (255), (int) (0), (int) (255)));  //lowEnd-purple
                        //Imgproc.line(contourImage, filtered.get(filtered.size() - 1), filtered.get(0), new Scalar((int) (255), (int) (0), (int) (0)), 3);
                        goodDetection = contourImage;
                        return goodDetection;
                    }
                    //endregion
                }
            }
            return input;
        }
        return goodDetection;
    }
    public double angleBtwn(Point a, Point b)
    {
        double angle = Math.toDegrees(Math.atan2(b.y - a.y, b.x - a.x));
        if (angle < 0)
        {
            angle += 360;
        }
        return angle;
    }
    public double distance(Point P, Point Q)
    {
        return Math.sqrt(((Q.x-P.x)*(Q.x-P.x)) + ((Q.y-P.y)*(Q.y-P.y)));
    }
    public pointVector getxLine()
    {
        return xLine;
    }
    public pointVector getyLine() {
        return yLine;
    }
    public pointVector getzLine() {
        return zLine;
    }
    public double getCenterLineOffset()
    {
        return centerLineOffset;
    }
    public String getArea(){
        return printable;
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