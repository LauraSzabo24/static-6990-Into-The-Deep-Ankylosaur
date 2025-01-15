package org.firstinspires.ftc.teamcode.drive.PinThings;
import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

public class pinpointLocalizer extends pinLocalizer {
    //region VARIABLES
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.6; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = 2.25; // X is the up and down direction 2.5
    public static double PARALLEL_Y = 6.5; // Y is the strafe direction 6.25

    public static double PERPENDICULAR_X = 5.5; //2.8
    public static double PERPENDICULAR_Y = 4; //5.25

    public static double X_MULTIPLIER = 0.9951095; //0.994657, 0.9966894, 0.993982
    public static double Y_MULTIPLIER = 1.0009521; //1.00405, 1.0011026, 1.0021226, 0.996078
    //endregion
    private NewMecanumDrive drive;
    GoBildaPinpointDriver odo;

    public pinpointLocalizer(HardwareMap hardwareMap, NewMecanumDrive drive) {
        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-50, -180); //-50 180
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
        this.drive = drive;
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return odo.getHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return odo.getHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                odo.getPosX()* X_MULTIPLIER,
                odo.getPosY() * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public Pose2d getOdoPosition() {
        odo.update();
        Vector2d input = new Vector2d(
                (odo.getPosX() * X_MULTIPLIER) * 0.0393701,
                (odo.getPosY() * Y_MULTIPLIER) * 0.0393701
        ).rotated(0);
        Pose2d odoPose = new Pose2d(input.getX(), input.getY(), odo.getHeading());
        //Pose2d odoPose = new Pose2d((odo.getPosX() * X_MULTIPLIER) *0.0393701, (-odo.getPosY() * Y_MULTIPLIER) * 0.0393701, odo.getHeading());
        return odoPose;
    }

    @NonNull
    @Override
    public Pose2d getOdoVelocity() {
        odo.update();
        Vector2d input = new Vector2d(
                (odo.getVelX() * X_MULTIPLIER)*0.0393701,
                (odo.getVelY() * Y_MULTIPLIER)*0.0393701
        ).rotated(0);
        Pose2d odoPose = new Pose2d(input.getX(), input.getY(), odo.getHeadingVelocity());

        //Pose2d odoPose = new Pose2d((odo.getVelX() * X_MULTIPLIER)*0.0393701, (-odo.getVelY() * Y_MULTIPLIER)*0.0393701, odo.getHeadingVelocity());
        return odoPose;
    }

    public void resetOdo() {
        odo.resetPosAndIMU();
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                odo.getVelX() * X_MULTIPLIER,
                -odo.getVelY() * Y_MULTIPLIER
        );
    }
}
