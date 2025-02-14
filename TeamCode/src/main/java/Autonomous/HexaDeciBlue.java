package Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class HexaDeciBlue extends OpMode {
    NewMecanumDrive drive;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    //region LED EFFECTS
    Gamepad.LedEffect resetField = new Gamepad.LedEffect.Builder()
            .addStep(0.5, 0.5, 0.5, 150)
            .addStep(0, 0, 0, 150)
            .addStep(0.5, 0.5, 0.5, 150)
            .addStep(0, 0, 0, 150)
            .addStep(0.5, 0.5, 0.5, 150)
            .addStep(0, 0, 0, 150)
            .build();
    ElapsedTime flashTimer = new ElapsedTime();

    //endregion

    //region FLIPPER CONTROLLER
    ElapsedTime timer = new ElapsedTime();
    private double flpPosError = 0;
    private double flpPosISum = 0;

    public static double flpPP = 10, flpPI = 0.05, flpPD = 0;
    public static int flpPosTarget = 0;
    //endregion

    //region EXTENDER CONTROLLER
    //POSITION
    private PIDController ext;
    //public static double extP = 0.005, extI = 0.03, extD = 0;
    public static double extP = 0.001, extI = 0.03, extD = 0;
    public static int extTarget;
    //VELOCITY
    ElapsedTime velTimer = new ElapsedTime();
    private double extVeloError = 0;
    private double extVeloISum = 0;
    public static double extVP = 0.001, extVI = 0, extVD = 0;
    public static int extVeloTarget = 0;
    //endregion

    //region DRIVER A MATERIAL
    Pose2d poseEstimate;
    private double speed;
    private double multiply;
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

    //region CONTROL STATE
    private enum poseControlState
    {
        FREE,
        PICKUP,
        VARIPICKUP,
        NEWWALL,
        OLDWALL,
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
    speedControlState gameModeA;
    speedControlState gameModeB;
    hangControlState hangState;
    poseControlState controlState;
    boolean editMode = true;
    boolean notNormalLimits = false;
    boolean variablePickup = false;
    public int flpLowLimit;
    public int flpHighLimit;
    //endregion

    //region TRAJECTORIES
    TrajectorySequence preload, preloadDrop, hexaBlock, hexaDrop,
            hexaPlop, cycleOne, dropOne, plopOne, cycleTwo, dropTwo,
            plopTwo, annoyingCycle, dropAnnoying, plopAnnoying,
            surpriseBlock, surpriseDrop, surprisePlop, park;
    Pose2d startPose, basketPose, onePose, preOnePose, twoPose,
            preTwoPose, threePose, preThreePose, hexPose, preHexPose,
            surprisePose, preSurprisePose, parkPose;
    boolean isVariablePickup = false;
    //endregion
    @Override
    public void init()
    {
        //region TECHNICALITIES
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Mailbox mail = new Mailbox();
        hardwareInit();
        startPose = new Pose2d(0,0,0);
        basketPose = new Pose2d(-19,5,0.8);

        preHexPose = new Pose2d(20,15,0);
        hexPose = new Pose2d(24,15,0);

        preOnePose = new Pose2d(-9.5,27,1.5656);
        onePose = new Pose2d(-9.5,37,1.5656);

        preTwoPose = new Pose2d(-19.5,27,1.5656);
        twoPose = new Pose2d(-19.5,37,1.5656);

        preThreePose = new Pose2d(-17,37.7163,3.16);
        threePose = new Pose2d(-25,37.7163,3.16);

        preSurprisePose = new Pose2d(14.4,56.7,3.16);
        surprisePose = new Pose2d(16,56.7,3.16);
        parkPose = new Pose2d(16,56.7,0);
        drive.setPoseEstimate(startPose);
        movementInitI();
        //endregion

        //region PRELOAD
        preload = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-5,5,0))
                .lineToLinearHeading(basketPose)
                .addTemporalMarker(0,() -> {
                    flpPosTarget = 0;
                    extTarget = 1520;
                })
                .addTemporalMarker(0.1,() -> {
                    spin.setPosition(0.1528);
                })
                .addTemporalMarker(0.4,() -> {
                    bigWristR.setPosition(0.6689);
                    bigWristL.setPosition(0.3278);
                    smallWrist.setPosition(0.5667);
                })
                .addTemporalMarker(1.5,() -> {drive.followTrajectorySequenceAsync(preloadDrop, mail);})
                .build();
        //endregion

        //region PRELOAD DROP
        preloadDrop = drive.trajectorySequenceBuilder(preload.end())
                .back(4)
                .addTemporalMarker(1,() -> {
                    claw.setPosition(0.9);
                })
                .addTemporalMarker(1.2,() -> {drive.followTrajectorySequenceAsync(hexaBlock, mail);})
                .build();
        //endregion

        //region HEXADECIMAL
        //region HEXABLOCK
        hexaBlock = drive.trajectorySequenceBuilder(preload.end())
                .addTemporalMarker(0,() -> {
                    bigWristR.setPosition(0.58);
                    bigWristL.setPosition(0.4);
                    smallWrist.setPosition(0.5667);
                })
                .lineToLinearHeading(preHexPose)
                .lineToLinearHeading(hexPose, NewMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), NewMecanumDrive.getAccelerationConstraint(30))
                .addTemporalMarker(0.3,() -> {
                    extTarget = 200;
                    smallWrist.setPosition(0.1367);
                    bigWristL.setPosition(0.95);
                    bigWristR.setPosition(0.05);
                })
                .addTemporalMarker(1,() -> {
                    smallWrist.setPosition(0.6306);
                    bigWristR.setPosition(0.04);
                    bigWristL.setPosition(0.96);
                })
                .addTemporalMarker(1.5,() -> {
                    extTarget = 0;
                })
                .addTemporalMarker(1.7,() -> {
                    extTarget = 60;
                    claw.setPosition(0.3);
                })
                .addTemporalMarker(1.8,() -> {drive.followTrajectorySequenceAsync(hexaDrop, mail);})
                .build();
        //endregion

        //region HEXADROP
        hexaDrop = drive.trajectorySequenceBuilder(hexaBlock.end())
                .lineToLinearHeading(basketPose)
                .addTemporalMarker(0,() -> {
                    flpPosTarget = 0;
                    extTarget = 1520;
                })
                .addTemporalMarker(0.1,() -> {
                    spin.setPosition(0.1528);
                })
                .addTemporalMarker(0.4,() -> {
                    bigWristR.setPosition(0.6689);
                    bigWristL.setPosition(0.3278);
                    smallWrist.setPosition(0.5667);
                })
                .addTemporalMarker(1.5,() -> {drive.followTrajectorySequenceAsync(hexaPlop, mail);})
                .build();
        //endregion

        //region HEXAPLOP
        hexaPlop = drive.trajectorySequenceBuilder(hexaDrop.end())
                .back(4)
                .addTemporalMarker(1,() -> {
                    claw.setPosition(0.9);
                })
                .addTemporalMarker(1.2,() -> {drive.followTrajectorySequenceAsync(cycleOne, mail);})
                .build();
        //endregion
        //endregion

        //region CYCLE ONE PARTS
        //region CYCLE ONE
        cycleOne = drive.trajectorySequenceBuilder(hexaPlop.end())
                .addTemporalMarker(0,() -> {
                    bigWristR.setPosition(0.58);
                    bigWristL.setPosition(0.4);
                    smallWrist.setPosition(0.5667);
                })
                .lineToLinearHeading(preOnePose)
                .lineToLinearHeading(onePose, NewMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), NewMecanumDrive.getAccelerationConstraint(30))
                .addTemporalMarker(0.3,() -> {
                    extTarget = 200;
                    smallWrist.setPosition(0.1367);
                    bigWristL.setPosition(0.95);
                    bigWristR.setPosition(0.05);
                })
                .addTemporalMarker(1,() -> {
                    smallWrist.setPosition(0.6306);
                    bigWristR.setPosition(0.04);
                    bigWristL.setPosition(0.96);
                })
                .addTemporalMarker(1.8,() -> {
                    extTarget = 60;
                    claw.setPosition(0.3);
                })
                .addTemporalMarker(1.9,() -> {drive.followTrajectorySequenceAsync(dropOne, mail);})
                .build();
        //endregion

        //region ONE DROP
        dropOne = drive.trajectorySequenceBuilder(cycleOne.end())
                .lineToLinearHeading(basketPose)
                .addTemporalMarker(0,() -> {
                    flpPosTarget = 0;
                    extTarget = 1540;
                })
                .addTemporalMarker(0.1,() -> {
                    spin.setPosition(0.1528);
                })
                .addTemporalMarker(0.4,() -> {
                    bigWristR.setPosition(0.6689);
                    bigWristL.setPosition(0.3278);
                    smallWrist.setPosition(0.5667);
                })
                .addTemporalMarker(1.5,() -> {drive.followTrajectorySequenceAsync(plopOne, mail);})
                .build();
        //endregion

        //region ONE PLOP
        plopOne = drive.trajectorySequenceBuilder(dropOne.end())
                .back(4)
                .addTemporalMarker(1,() -> {
                    claw.setPosition(0.9);
                })
                .addTemporalMarker(1.2,() -> {drive.followTrajectorySequenceAsync(cycleTwo, mail);})
                .build();
        //endregion
        //endregion

        //region CYCLE TWO PARTS
        //region CYCLE TWO
        cycleTwo = drive.trajectorySequenceBuilder(plopOne.end())
                .addTemporalMarker(0,() -> {
                    bigWristR.setPosition(0.58);
                    bigWristL.setPosition(0.4);
                    smallWrist.setPosition(0.5667);
                })
                .lineToLinearHeading(preTwoPose)
                .lineToLinearHeading(twoPose, NewMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), NewMecanumDrive.getAccelerationConstraint(30))
                .addTemporalMarker(0.3,() -> {
                    extTarget = 200;
                    smallWrist.setPosition(0.1367);
                    bigWristL.setPosition(0.95);
                    bigWristR.setPosition(0.05);
                })
                .addTemporalMarker(1,() -> {
                    smallWrist.setPosition(0.6306);
                    bigWristR.setPosition(0.04);
                    bigWristL.setPosition(0.96);
                })
                .addTemporalMarker(1.8,() -> {
                    extTarget = 60;
                    claw.setPosition(0.3);
                })
                .addTemporalMarker(1.9,() -> {drive.followTrajectorySequenceAsync(dropTwo, mail);})
                .build();
        //endregion

        //region TWO DROP
        dropTwo = drive.trajectorySequenceBuilder(cycleTwo.end())
                .lineToLinearHeading(basketPose)
                .addTemporalMarker(0,() -> {
                    flpPosTarget = 0;
                    extTarget = 1560;
                })
                .addTemporalMarker(0.1,() -> {
                    spin.setPosition(0.1528);
                })
                .addTemporalMarker(0.4,() -> {
                    bigWristR.setPosition(0.6689);
                    bigWristL.setPosition(0.3278);
                    smallWrist.setPosition(0.5667);
                })
                .addTemporalMarker(1.5,() -> {drive.followTrajectorySequenceAsync(plopTwo, mail);})
                .build();
        //endregion

        //region TWO PLOP
        plopTwo = drive.trajectorySequenceBuilder(dropTwo.end())
                .back(4)
                .addTemporalMarker(1,() -> {
                    claw.setPosition(0.9);
                })
                .addTemporalMarker(1.2,() -> {drive.followTrajectorySequenceAsync(annoyingCycle, mail);})
                .build();
        //endregion
        //endregion

        //region ANNOYING PARTS
        //region ANNOYING CYCLE
        annoyingCycle = drive.trajectorySequenceBuilder(plopTwo.end())
                .addTemporalMarker(0,() -> {
                    bigWristR.setPosition(0.58);
                    bigWristL.setPosition(0.4);
                    smallWrist.setPosition(0.5667);
                })
                .lineToLinearHeading(preThreePose)
                .lineToLinearHeading(threePose, NewMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), NewMecanumDrive.getAccelerationConstraint(30))
                .addTemporalMarker(0.3,() -> {
                    extTarget = 230;
                    smallWrist.setPosition(0.1367);
                    bigWristL.setPosition(0.95);
                    bigWristR.setPosition(0.05);
                })
                .addTemporalMarker(1,() -> {
                    smallWrist.setPosition(0.6306);
                    bigWristR.setPosition(0.04);
                    bigWristL.setPosition(0.96);
                    spin.setPosition(0.4239);
                })
                .addTemporalMarker(2.1,() -> {
                    extTarget = 0;
                    claw.setPosition(0.3);
                })
                .addTemporalMarker(2.2,() -> {drive.followTrajectorySequenceAsync(dropAnnoying, mail);})
                .build();
        //endregion

        //region ANNOYING DROP
        dropAnnoying = drive.trajectorySequenceBuilder(annoyingCycle.end())
                .lineToLinearHeading(basketPose)
                .addTemporalMarker(0,() -> {
                    flpPosTarget = 0;
                    extTarget = 1560;
                })
                .addTemporalMarker(0.1,() -> {
                    spin.setPosition(0.1528);
                })
                .addTemporalMarker(0.4,() -> {
                    bigWristR.setPosition(0.6689);
                    bigWristL.setPosition(0.3278);
                    smallWrist.setPosition(0.5667);
                })
                .addTemporalMarker(1.5,() -> {drive.followTrajectorySequenceAsync(plopAnnoying, mail);})
                .build();
        //endregion

        //region ANNOYING PLOP
        plopAnnoying = drive.trajectorySequenceBuilder(dropAnnoying.end())
                .back(4)
                .addTemporalMarker(1,() -> {
                    claw.setPosition(0.9);
                })
                .addTemporalMarker(1.2,() -> {drive.followTrajectorySequenceAsync(surpriseBlock, mail);})
                .build();
        //endregion
        //endregion

        //region SURPRISE PARTS
        //region SURPRISE BLOCK
        surpriseBlock = drive.trajectorySequenceBuilder(plopAnnoying.end())
                .addTemporalMarker(0,() -> {
                    bigWristR.setPosition(0.58);
                    bigWristL.setPosition(0.4);
                    smallWrist.setPosition(0.5667);
                })
                .lineToLinearHeading(preSurprisePose)
                .lineToLinearHeading(surprisePose, NewMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), NewMecanumDrive.getAccelerationConstraint(30))

                .waitSeconds(1)
                .addTemporalMarker(0.3,() -> {
                    extTarget = 0;
                    spin.setPosition(0.7106);
                    bigWristR.setPosition(0);
                    bigWristL.setPosition(1);
                    smallWrist.setPosition(0.1367);
                })
                .addTemporalMarker(1.2,() -> {
                    flpPosTarget = 1700;
                })
                .addTemporalMarker(1.5,() -> {
                    spin.setPosition(0.7472);
                    bigWristR.setPosition(0.52);
                    bigWristL.setPosition(0.4789);
                    smallWrist.setPosition(0.3489);
                    extTarget = 40;
                })
                .addTemporalMarker(1.7,() -> {
                    claw.setPosition(0.9);
                    isVariablePickup = true;
                })
                //.addTemporalMarker(1.9,() -> {drive.followTrajectorySequenceAsync(surpriseDrop, mail);})
                .build();
        //endregion

        //region SURPRISE DROP
        surpriseDrop = drive.trajectorySequenceBuilder(surpriseBlock.end())
                .lineToLinearHeading(basketPose)
                .addTemporalMarker(0,() -> {
                    flpPosTarget = 0;
                    extTarget = 1520;
                })
                .addTemporalMarker(0.1,() -> {
                    spin.setPosition(0.1528);
                })
                .addTemporalMarker(0.4,() -> {
                    bigWristR.setPosition(0.6689);
                    bigWristL.setPosition(0.3278);
                    smallWrist.setPosition(0.5667);
                })
                .addTemporalMarker(1.5,() -> {drive.followTrajectorySequenceAsync(plopOne, mail);})
                .build();
        //endregion

        //region SURPRISE PLOP
        surprisePlop = drive.trajectorySequenceBuilder(surpriseDrop.end())
                .back(4)
                .addTemporalMarker(1,() -> {
                    claw.setPosition(0.9);
                })
                .addTemporalMarker(1.2,() -> {drive.followTrajectorySequenceAsync(park, mail);})
                .build();
        //endregion
        //endregion

        //region PARK
        park = drive.trajectorySequenceBuilder(surprisePlop.end())
                .addTemporalMarker(0,() -> {
                    bigWristR.setPosition(0.58);
                    bigWristL.setPosition(0.4);
                    smallWrist.setPosition(0.5667);
                })
                .lineToLinearHeading(parkPose)

                .waitSeconds(1)
                .addTemporalMarker(0.3,() -> {
                    extTarget = 0;
                    spin.setPosition(0.7106);
                    bigWristR.setPosition(0);
                    bigWristL.setPosition(1);
                    smallWrist.setPosition(0.1367);
                })
                .addTemporalMarker(1,() -> {
                    spin.setPosition(0.7472);
                    bigWristR.setPosition(0.52);
                    bigWristL.setPosition(0.4789);
                    smallWrist.setPosition(0.3489);
                    extTarget = 200;
                })
                .build();
        //endregion

        drive.followTrajectorySequenceAsync(preload, mail);
        mail.setAutoEnd((new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())));
    }
    @Override
    public void loop()  {
        drive.update();
        poseEstimate = new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading());
        extOldCONTROLLER();
        flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
        telemetry.addData("CURR HEADING", drive.getPoseEstimate().getHeading());
        telemetry.addData("XPos", poseEstimate.getX());
        telemetry.addData("YPos", poseEstimate.getY());
        telemetry.update();

        if(controlState == poseControlState.PICKUP)
        {
            extPercentage = (extTarget-40.0)/1120;
            spin.setPosition(0.1528);
            smallWrist.setPosition(0.1706 + ((0.2556-0.1706)*extPercentage));
            bigWristL.setPosition(0.6989 + ((0.7389-0.6989)*extPercentage));
            bigWristR.setPosition(0.3 - ((0.3-0.26)*extPercentage));
            //EXT          40         1120
            //smallWrist - 0.1706 - 0.2556
            //bigWristL - 0.6989 - 0.7389
            //bigWristR - 0.3 - 0.26
        }
    }
    public void hardwareInit()
    {
        //RANDOM
        dashboard = FtcDashboard.getInstance();

        //PID
        ext = new PIDController(extP, extI, extD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //arm motors
        flipMotor = hardwareMap.get(DcMotorEx.class, "FLIP");
        flipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flipMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extLMotor = hardwareMap.get(DcMotorEx.class, "EL");
        extLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extRMotor = hardwareMap.get(DcMotorEx.class, "ER");
        extRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //drive motors
        drive = new NewMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(Mailbox.currentPose);

        //servos
        smallWrist = hardwareMap.get(Servo.class, "SW");
        bigWristL = hardwareMap.get(Servo.class, "BWL");
        bigWristR = hardwareMap.get(Servo.class, "BWR");
        spin = hardwareMap.get(Servo.class, "SPIN");
        claw = hardwareMap.get(Servo.class, "CLAW");
    }
    public void movementInitI()
    {
        extTarget = 0;
        bigWristL.setPosition(0.2078);
        bigWristR.setPosition(0.76);
        smallWrist.setPosition(0.1367);
        claw.setPosition(0.35);
        spin.setPosition(0.725);
    }
    public void extOldCONTROLLER()
    {
        ext.setPID(extP, extI, extD);
        int extPose = -extLMotor.getCurrentPosition();
        double extPwr = ext.calculate(extPose, extTarget);
        extLMotor.setPower(-extPwr);
        extRMotor.setPower(extPwr);

        telemetry.addData("extPos ", extPose);
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
        telemetry.addData("FLIP VELO", velocityVal);
        if(velocityVal>1700){
            velocityVal=1700;
        }
        else if (velocityVal<-1700)
        {
            velocityVal = -1700;
        }
        flipMotor.setVelocity(velocityVal);
    }
    public void RAINBOW(double speed)
    {
        for(double i=0; i<1; i+=0.01)
        {
            jerkTimer.reset();
            while(jerkTimer.time() < speed) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            gamepad1.setLedColor(1, i, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(1, i, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
        for(double i=1; i>0; i-=0.01)
        {
            jerkTimer.reset();
            while(jerkTimer.time() < speed) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            gamepad1.setLedColor(1, i, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(i, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
        for(double i=0; i<1; i+=0.01)
        {
            jerkTimer.reset();
            while(jerkTimer.time() < speed) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            gamepad1.setLedColor(1, i, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(0, 1, i, Gamepad.LED_DURATION_CONTINUOUS);
        }
        for(double i=1; i>0; i-=0.01)
        {
            jerkTimer.reset();
            while(jerkTimer.time() < speed) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            gamepad1.setLedColor(1, i, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(0, i, 1, Gamepad.LED_DURATION_CONTINUOUS);
        }
        for(double i=0; i<1; i+=0.01)
        {
            jerkTimer.reset();
            while(jerkTimer.time() < speed) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            gamepad1.setLedColor(1, i, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(i, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        }
        for(double i=1; i>0; i-=0.01)
        {
            jerkTimer.reset();
            while(jerkTimer.time() < speed) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            gamepad1.setLedColor(1, i, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(1, 0, i, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }
}