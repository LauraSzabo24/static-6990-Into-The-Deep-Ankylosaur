package Autonomous.Detectors;


import org.opencv.core.Point;

public class pointVector {

    Point P;
    Point Q;

    double magnitude;
    public pointVector(Point a, Point b, double mag)
    {
        P = a;
        Q = b;
        magnitude = mag;
    }

    public double getMagnitude()
    {
        return magnitude;
    }
    public Point getP()
    {
        return P;
    }
    public Point getQ()
    {
        return Q;
    }
}
