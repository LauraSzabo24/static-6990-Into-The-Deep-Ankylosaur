package RelicsFromCenterstageGame.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Color {
        WHITE,
        PURPLE,
        YELLOW,
        GREEN,
        NOTHING
    }
    private Color pixelColor;

    //for reference only (0,0) in top left
    static final Rect SCREENSIZEBOX = new Rect( //make this the correct area
            new Point(0, 0),
            new Point(320, 240));

    //actual boxes
    static final Rect INTAKEBOX = new Rect( //make this the correct area
            new Point(0, 0),
            new Point(320, 240));
    static double PERCENT_COLOR_THRESHOLD = 0.2;


    public ColorDetector(Telemetry t)    {telemetry = t;    }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        //COLORS
        Scalar lowHSVYELLOW= new Scalar(10, 0, 0);
        Scalar highHSVYELLOW = new Scalar(25, 255, 255);

        Scalar lowHSVGREEN= new Scalar(45, 0, 0);
        Scalar highHSVGREEN = new Scalar(76, 255, 255);

        Scalar lowHSVPURPLE= new Scalar(120, 20, 50);
        Scalar highHSVPURPLE = new Scalar(170, 200, 255);

        Scalar lowHSVWHITE= new Scalar(0, 0, 80);
        Scalar highHSVWHITE = new Scalar(180, 30, 255);

        pixelColor = Color.NOTHING;
        Mat newMat = mat;
        boolean run  = false;

        //purple
        Core.inRange(mat, lowHSVPURPLE, highHSVPURPLE, newMat);
        Mat purpleIntakeArea = mat.submat(INTAKEBOX);
        double purpleColorPercentage = Core.sumElems(purpleIntakeArea).val[0] / INTAKEBOX.area() / 255;
        purpleIntakeArea.release();
        if(purpleColorPercentage > PERCENT_COLOR_THRESHOLD && run==false)
        {
            run = true;
            pixelColor = Color.PURPLE;
            telemetry.addData("in Color", "purple");
        }
        //yellow
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(mat, lowHSVYELLOW, highHSVYELLOW, newMat);
        Mat yellowIntakeArea = mat.submat(INTAKEBOX);
        double yellowColorPercentage = Core.sumElems(yellowIntakeArea).val[0] / INTAKEBOX.area() / 255;
        yellowIntakeArea.release();
        if(yellowColorPercentage > PERCENT_COLOR_THRESHOLD && run==false)
        {
            run = true;
            pixelColor = Color.YELLOW;
            telemetry.addData("in Color", "yellow");
        }
        //green
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(mat, lowHSVGREEN, highHSVGREEN, newMat);
        Mat greenIntakeArea = mat.submat(INTAKEBOX);
        double greenColorPercentage = Core.sumElems(greenIntakeArea).val[0] / INTAKEBOX.area() / 255;
        greenIntakeArea.release();
        if(greenColorPercentage > PERCENT_COLOR_THRESHOLD && run==false)
        {
            run = true;
            pixelColor = Color.GREEN;
            telemetry.addData("in Color", "green");
        }
        //white
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(mat, lowHSVWHITE, highHSVWHITE, newMat);
        Mat whiteIntakeArea = mat.submat(INTAKEBOX);
        double whiteColorPercentage = Core.sumElems(whiteIntakeArea).val[0] / INTAKEBOX.area() / 255;
        whiteIntakeArea.release();
        if(whiteColorPercentage > PERCENT_COLOR_THRESHOLD && run==false)
        {
            run = true;
            pixelColor = Color.WHITE;
            telemetry.addData("in Color", "white");
        }

        if(pixelColor == null || pixelColor == Color.NOTHING)
        {
            pixelColor = Color.NOTHING;
            telemetry.addData("in Color", "nothing");
        }
        telemetry.update();

        //for display purposes
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar propBoxColor = new Scalar(255, 0, 0); //for box drawing purposes

        //draws rectangle
        Imgproc.rectangle(mat, INTAKEBOX, propBoxColor);

        return mat;
    }
    public Color getColor()
    {
        return pixelColor;
    }
}
