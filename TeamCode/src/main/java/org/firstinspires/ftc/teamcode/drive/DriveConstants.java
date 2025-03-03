package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class DriveConstants {

    public static final double TICKS_PER_REV = 537.7;
    public static final double MAX_RPM = 312;

    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(8, 0, -0.3,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    public static double WHEEL_RADIUS = 2.04724; // in
    public static double GEAR_RATIO =(.75* (62/48.13) * (46.5/44.71)) *1.2105; //.75* output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 16.65; // in

    public static double kV = 0.0153;
    public static double kA = 0.0034;
    public static double kStatic = 0.0025;


    public static double MAX_VEL = 60; //30 80
    public static double MAX_ACCEL = 30; //30
    public static double MAX_ANG_VEL = Math.toRadians(162.31);
    public static double MAX_ANG_ACCEL = Math.toRadians(60);

    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static void setSlowSpeed()
    {
        MAX_VEL = 5; //30 80
        MAX_ACCEL = 5; //30
    }
    public static void setRegularSpeed()
    {
        MAX_VEL = 70; //30 80
        MAX_ACCEL = 70; //30
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}