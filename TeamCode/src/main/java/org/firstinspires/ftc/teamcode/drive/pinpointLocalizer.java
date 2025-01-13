package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

public class pinpointLocalizer extends TwoTrackingWheelLocalizer {
    //region VARIABLES
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.6; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
    public static double PARALLEL_X = 2.25; // X is the up and down direction 2.5
    public static double PARALLEL_Y = 6.5; // Y is the strafe direction 6.25
    public static double PERPENDICULAR_X = 5.5; //2.8
    public static double PERPENDICULAR_Y = 4; //5.25
    //endregion
    public static double X_MULTIPLIER = 1.173 * 0.989; //77.034, 76.93, 76.2
    public static double Y_MULTIPLIER = 1.1586 * 0.9977; //77.698, 77.92, 77.42
    GoBildaPinpointDriver odo;
    public pinpointLocalizer(HardwareMap hardwareMap, NewMecanumDrive drive) {
        super(Arrays.asList(
            new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
            new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-3, -26);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
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
                odo.getPosX() * X_MULTIPLIER,
                odo.getPosY() * Y_MULTIPLIER
        );
    }
    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                odo.getVelX() * X_MULTIPLIER,
                odo.getVelY() * Y_MULTIPLIER
        );
    }
}
