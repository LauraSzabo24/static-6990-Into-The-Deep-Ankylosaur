package Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.drive.Unused.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

@Autonomous
public class Red4Specimen extends OpMode {

    //region LED EFFECTS
    Gamepad.LedEffect resetField = new Gamepad.LedEffect.Builder()
            .addStep(0.5, 0.5, 0.5, 150)
            .addStep(0, 0, 0, 150)
            .addStep(0.5, 0.5, 0.5, 150)
            .addStep(0, 0, 0, 150)
            .addStep(0.5, 0.5, 0.5, 150)
            .addStep(0, 0, 0, 150)
            .build();
    //endregion

    //region FLIPPER CONTROLLER
    //POSITION
    ElapsedTime timer = new ElapsedTime();
    private double flpPosError = 0;
    private double flpPosISum = 0;

    public static double flpPP = 2.5, flpPI = 0, flpPD = 0.1;
    public static int flpPosTarget = 0;
    //endregion

    //region EXTENDER CONTROLLER
    public static double ticksPerDegree = 537.7;
    private PIDController ext;
    public static double extP = 0.005, extI = 0.03, extD = 0.00035;
    public static int extTarget;
    FtcDashboard dashboard;
    //endregion

    //region DRIVER A MATERIAL
    NewMecanumDrive drive;
    Pose2d poseEstimate;
    private double speed;
    private double multiply;
    GoBildaPinpointDriver odo;
    double oldTime = 0;
    IMU imu;
    IMU.Parameters parameters;
    double extPercentage;
    //endregion

    //region DRIVER B MATERIAL
    private Servo smallWrist, bigWristR, bigWristL, spin, claw;
    DcMotorEx flipMotor, extLMotor, extRMotor;
    boolean clawIH;
    ElapsedTime jerkTimer = new ElapsedTime();
    int spinnerPos = 0;
    int smallWristPos = 0;
    //endregion

    //region GAMEPADS
    Gamepad currG1;
    Gamepad oldG1;
    Gamepad currG2;
    Gamepad oldG2;
    //endregion

    //region CONTROL STATE
    private enum poseControlState
    {
        FREE,
        PICKUP,
        WALL,
        LOW,
        HIGH,
        HOME,
        LOWBASKET,
        HIGHBASKET

    }
    private enum speedControlState
    {
        NORMAL,
        PRECISION,
        SUPERSPEED
    }
    private enum hangControlState
    {
        PHASEONE,
        PHASETWO,
        CONFIRMATION,
        DISENGAGE
    }
    private speedControlState gameModeA;
    private speedControlState gameModeB;
    private hangControlState hangState;
    private poseControlState controlState;
    boolean editMode = true;
    boolean notNormalLimits = false;
    //endregion
    TrajectorySequence preload, connection, cycleOne, cycleTwo, cycleThree;
    @Override
    public void init()
    {
        telemetry.setMsTransmissionInterval(50);
        Mailbox mail = new Mailbox();
        hardwareInit();
        movementInitII();
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        TrajectoryVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(25),
                new AngularVelocityConstraint(1)
        ));

        //region PRELOAD DROP OFF
        preload = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(-10, 26), Math.toRadians(90)) //29
                .addDisplacementMarker(1,() -> {
                    extTarget = 1600;
                    flpPosTarget = -1400;
                })
                .addDisplacementMarker(20,() -> {
                    //spinnerServo.setPosition(0.77 + 0.1217 - 0.0528);
                    //wristServo.setPosition(0.95);//0.8389
                })
                .addTemporalMarker(2.5,() -> {
                    //spinnerServo.setPosition(0.77 + 0.1217 - 0.0528);
                    //wristServo.setPosition(1);
                    flpPosTarget = -1300;
                    extTarget = 1140;
                })
                .addTemporalMarker(3,() -> {
                    //clawServo.setPosition(0);
                })
                .waitSeconds(2)
                .addTemporalMarker(3.5,() -> {drive.followTrajectorySequenceAsync(connection, mail);})
                .build();
        //endregion

        //region CONNECTION
        connection = drive.trajectorySequenceBuilder(preload.end())
                .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(90))
                .setVelConstraint(slowConstraint)
                .splineToConstantHeading(new Vector2d(15, 30), Math.toRadians(90))
                .resetConstraints()
                .splineToConstantHeading(new Vector2d(20, 40), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(30, 52), Math.toRadians(90))
                .setVelConstraint(slowConstraint)
                .splineToConstantHeading(new Vector2d(30, 15), Math.toRadians(90))

                .resetConstraints()
                .splineToConstantHeading(new Vector2d(32, 40), Math.toRadians(90))
                .setVelConstraint(slowConstraint)
                .splineToConstantHeading(new Vector2d(39, 52), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(39, 30), Math.toRadians(90))
                .waitSeconds(2)
                .splineToConstantHeading(new Vector2d(39, 16), Math.toRadians(90))

                .addTemporalMarker(8,() -> {
                    extTarget = 250;
                    flpPosTarget = -3760; //3800
                    //clawServo.setPosition(0);
                })
                .addTemporalMarker(8.5,() -> {
                    //spinnerServo.setPosition(0.20 + 0.1217 - 0.0528); //0.215
                    //wristServo.setPosition(0.6);
                })

                .addTemporalMarker(12,() -> {
                    //clawServo.setPosition(0.4);
                })
                .addTemporalMarker(14,() -> {
                    extTarget = 500;
                    flpPosTarget = -3600;
                })
                .waitSeconds(3)
                .addTemporalMarker(17,() -> {drive.followTrajectorySequenceAsync(cycleOne, mail);})
                .build();
        //endregion

        //region CYCLE ONE
        cycleOne = drive.trajectorySequenceBuilder(connection.end())
                .splineToConstantHeading(new Vector2d(-16, 32), Math.toRadians(90))
                .addDisplacementMarker(1,() -> {
                    extTarget = 1700;
                    flpPosTarget = -1900;
                })
                .addDisplacementMarker(20,() -> {
                    //spinnerServo.setPosition(0.77 + 0.1217 - 0.0528);
                    //wristServo.setPosition(0.85);//0.8389
                })
                .addTemporalMarker(4,() -> {
                    //spinnerServo.setPosition(0.77 + 0.1217 - 0.0528);
                    //wristServo.setPosition(1);
                    flpPosTarget = -1400;
                    extTarget = 1140;
                })
                .addTemporalMarker(5,() -> {
                    //clawServo.setPosition(0);
                })
                .waitSeconds(2)

                .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(39, 6), Math.toRadians(90))
                .addTemporalMarker(8,() -> {
                    extTarget = 0;
                })
                .addTemporalMarker(9,() -> {
                    //wristServo.setPosition(0.39);
                    //spinnerServo.setPosition(0.21 + 0.1217 - 0.0528);
                    //clawServo.setPosition(0.4);
                    flpPosTarget = 0;
                })
                .build();
        //endregion

        //DRIVING
        drive.setPoseEstimate(startPose);
        drive.followTrajectorySequenceAsync(preload, mail);
        mail.setAutoEnd((new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading() + Math.toRadians(-180))));
    }
    @Override
    public void loop()  {
        drive.update();
        extCONTROLLER();
        flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
    }
    public void highPosition()
    {
        telemetry.addLine("TO HIGH POSITION");
        if(flpPosTarget>-2000) {
            extTarget = 1600;
            flpPosTarget = -1650;
            jerkTimer.reset();
            while(jerkTimer.time() < 0.5) {
            }
            //spinnerServo.setPosition(0.77 + 0.1217 - 0.0528);
            //wristServo.setPosition(0.8389);
        }
        else {
            /*jerkTimer.reset();
            while(jerkTimer.time() < 0.5) {
                extTarget = 0;
                extCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            armMotor.setPower(0);

            spinnerServo.setPosition(0.77);
            wristServo.setPosition(0.8389);
            jerkTimer.reset();
            while(jerkTimer.time() < 0.5) {
                flpPosTarget = -1650;
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            jerkTimer.reset();
            while(jerkTimer.time() < 0.5) {
                extTarget = 1600;
                extCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());

            }*/
        }
    }
    public void lowPosition()
    {
        jerkTimer.reset();
        while(jerkTimer.time() < 0.5) {
            extTarget = 0;
            extCONTROLLER();
            flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
        }
        //armMotor.setPower(0);

        //spinnerServo.setPosition(0.77 + 0.1217 - 0.0528);
        //wristServo.setPosition(0.7989);
        jerkTimer.reset();
        while(jerkTimer.time() < 0.5) {
            flpPosTarget = -1160;
            flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
        }
        jerkTimer.reset();
        while(jerkTimer.time() < 0.5) {
            extTarget = 440;
            extCONTROLLER();
            flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());

        }
    }
    public void homePosition()
    {
        telemetry.addLine("TO HOME POSITION");
        jerkTimer.reset();
        while(jerkTimer.time() < 0.5) {
            extTarget = 250;
            extCONTROLLER();
            flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
        }

        //wristServo.setPosition(0.39);
        //spinnerServo.setPosition(0.21 + 0.1217 - 0.0528);
        //clawServo.setPosition(0.4);
        jerkTimer.reset();
        while(jerkTimer.time() < 0.5) {
            flpPosTarget = -200;
            flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
        }
    }
    public void pickupPosition()
    {
        telemetry.addLine("TO PICKUP POSITION");
        jerkTimer.reset();
        while(jerkTimer.time() < 0.5) {
            extTarget = 250;
            extCONTROLLER();
            flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
        }
        //armMotor.setPower(0);
        //spinnerServo.setPosition(0.215 + 0.1217 - 0.0528);
        //wristServo.setPosition(0.66);
        //clawServo.setPosition(0);
        jerkTimer.reset();
        while (jerkTimer.time() < 0.5) {
            flpPosTarget = -3671;
            flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
        }
    }
    public void jerk()
    {
        while (jerkTimer.time() < 3) {
        }
        //spinnerServo.setPosition(0.77 + 0.1217 - 0.0528);
        //wristServo.setPosition(0.95);
        extTarget = 1140;
        jerkTimer.reset();
        while (jerkTimer.time() < 0.6) {
        }
        //clawServo.setPosition(0);
    }
    public void extCONTROLLER()
    {
        ext.setPID(extP, extI, extD);
        //int extPose = armMotor.getCurrentPosition();
        //double extPwr = ext.calculate(extPose, extTarget) + (Math.cos(Math.toRadians(extTarget/ticksPerDegree)) * extF);
        //armMotor.setPower(extPwr *(1/3.0));

        //telemetry.addData("extPos ", extPose);
        telemetry.addData("extTarget ", extTarget);
    }
    public void flpCONTROLLER(int target, int state) //in with the target -> out with the velocity
    {
        int currError = target - state;
        double time = timer.seconds();
        timer.reset();
        flpPosISum += currError * time;
        double deriv = (currError - flpPosError)/time;
        flpPosError = currError;

        double velocityVal = (flpPP * currError) + (flpPI * flpPosISum) + (flpPD*deriv);
        flipMotor.setVelocity(velocityVal);

        /*double veloTarget = (flpPP * currError) + (flpPI * flpPosISum) + (flpPD*deriv);
        telemetry.addData("targetvelo",veloTarget);
        double neededPower = flpVelocityCONTROLLER(veloTarget, flipMotor.getVelocity(), time);
        telemetry.addData("power",neededPower);
        flipMotor.setPower(neededPower);*/
    }
    public void hardwareInit()
    {
        //RANDOM
        dashboard = FtcDashboard.getInstance();

        //PID
        ext = new PIDController(extP, extI, extD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        //arm motors
        flipMotor = hardwareMap.get(DcMotorEx.class, "flip");
        flipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flipMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //drive motors
        drive = new NewMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(Mailbox.currentPose);

        //servos
        //wristServo = hardwareMap.get(Servo.class, "wrist");
        //spinnerServo = hardwareMap.get(Servo.class, "spinner");
        //clawServo = hardwareMap.get(Servo.class, "claw");

        //mailbox
        imu = hardwareMap.get(IMU.class, "imu");
        parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }
    public void movementInitII()
    {
        for(int i=0; i<15; i++)
        {
            //spinnerServo.setPosition(0.21 + 0.1217 - 0.0528);
            //wristServo.setPosition(0.39);
            //clawServo.setPosition(0.4);
        }
        flpPosTarget = -200;
        extTarget = 0;
    }

}

//0.21 0.3317  0.1217