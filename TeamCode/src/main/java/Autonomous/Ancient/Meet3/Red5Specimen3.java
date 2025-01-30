package Autonomous.Ancient.Meet3;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class Red5Specimen3 extends OpMode {
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

    public static double flpPP = 2.5, flpPI = 0, flpPD = 0;
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
    private enum poseControlState {
        FREE,
        PICKUP,
        NEWWALL,
        OLDWALL,
        LOW,
        HIGH,
        HOME,
        LOWBASKET,
        HIGHBASKET

    }
    poseControlState controlState;
    boolean firstRun;
    //endregion

    //region TRAJECTORIES
    TrajectorySequence preset, preload, dropOffI1, dropOffI2, dropOffI3, dropOffII1, dropOffII2, dropOffII3, dropOffIII, cycleI, cycleII, cycleIII, cycleIV;
    //endregion

    @Override
    public void init()
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Mailbox mail = new Mailbox();
        hardwareInit();
        Pose2d startPose = new Pose2d(0,0,0);
        drive.setPoseEstimate(startPose);
        extTarget = 0;
        flpPosTarget = 0;

        preset = drive.trajectorySequenceBuilder(startPose)
                .back(1)
                .forward(1)
                .waitSeconds(2)
                .addTemporalMarker(0.5,() -> {
                    movementInitI();
                })
                .addTemporalMarker(5.5,() -> {drive.followTrajectorySequenceAsync(preload, mail);})
                .build();

        //region PRELOAD & PICK UP GOOD
        preload = drive.trajectorySequenceBuilder(preset.end())
               .lineTo(new Vector2d(32, 13))
                .addTemporalMarker(0,() -> {
                    extTarget = 1260;//1060
                    spin.setPosition(0.1567);
                    bigWristR.setPosition(0.88);
                    bigWristL.setPosition(0.1194);
                    smallWrist.setPosition(0.1567);
                })
                .addTemporalMarker(0.5,() -> {
                    smallWrist.setPosition(0.05);
                })
                .addTemporalMarker(1.5,() -> {
                    smallWrist.setPosition(0.17);
                })
               .addTemporalMarker(1.8, this::jerk)

                .addTemporalMarker(2.8,() -> {
                    extTarget = 0;
                })
                .addTemporalMarker(3.6,() -> {
                    spin.setPosition(0.1567);
                    bigWristR.setPosition(0.88);
                    bigWristL.setPosition(0.1194);
                    smallWrist.setPosition(0.1567);
                })

                .lineTo(new Vector2d(20, 5))
                .splineToConstantHeading(new Vector2d(31, -32), Math.toRadians(0), NewMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), NewMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .addTemporalMarker(4.9, this::miniPickup)
                .addTemporalMarker(5,() -> {
                    claw.setPosition(0.6);
                    bigWristR.setPosition(0.81);
                    bigWristL.setPosition(0.19);
                })
                .lineTo(new Vector2d(20, -37))
                .addTemporalMarker(6,() -> {drive.followTrajectorySequenceAsync(dropOffI1, mail);})
                .waitSeconds(10)
                .build();
        //endregion

        //region DROP OFF I 1
        dropOffI1 = drive.trajectorySequenceBuilder(preload.end())
                //go to old wall position
                .lineTo(new Vector2d(15, -37))
                .waitSeconds(10)
                .addTemporalMarker(0,() -> {
                    controlState = poseControlState.OLDWALL;
                    extTarget = 40;
                    flpPosTarget = 1860;
                    smallWrist.setPosition(0.3206);
                    spin.setPosition(0.7106);
                })
                .addTemporalMarker(0.7,() -> {drive.followTrajectorySequenceAsync(dropOffI2, mail);})
                .build();
        //endregion

        //region DROP OFF I 2
        dropOffI2 = drive.trajectorySequenceBuilder(dropOffI1.end())
                .waitSeconds(1)
                .back(4)
                .waitSeconds(10)
                //far down
                .addTemporalMarker(0,() -> {
                    for(int i=0; i<100; i++) {
                        bigWristR.setPosition(0.45);
                        bigWristL.setPosition(0.55);
                        smallWrist.setPosition(0.4);
                    }
                })

                .addTemporalMarker(1,() -> {
                    claw.setPosition(0.6);
                })
                .addTemporalMarker(2,() -> {drive.followTrajectorySequenceAsync(dropOffI3, mail);})
                .build();
        //endregion

        //region DROP OFF I 3
        dropOffI3 = drive.trajectorySequenceBuilder(dropOffI2.end())
                .back(3)
                .waitSeconds(10)
                //close and lift
                .addTemporalMarker(0,() -> {
                    for(int i=0; i<100; i++) {
                        claw.setPosition(0.3);
                    }
                })
                .addTemporalMarker(1.5,() -> {
                    for(int i=0; i<100; i++) {
                        claw.setPosition(0.6);
                    }
                })
                .addTemporalMarker(2,() -> {
                    extTarget = 0;
                    for(int i=0; i<100; i++) {
                        bigWristR.setPosition(0.5394);
                        bigWristL.setPosition(0.46);
                    }
                })
                .addTemporalMarker(3.5,() -> {drive.followTrajectorySequenceAsync(cycleI, mail);})
                .build();
        //endregion

        //region CYCLE I
        cycleI = drive.trajectorySequenceBuilder(dropOffI3.end())
                .lineTo(new Vector2d(25, 17))
                .waitSeconds(0.1)
                .lineTo(new Vector2d(32, 17))
                .waitSeconds(0.3)

                //to high position
                .addTemporalMarker(0,() -> {
                    flpPosTarget = -100;
                })
                .addTemporalMarker(0,() -> {
                    for(int i=0; i<100; i++) {
                        spin.setPosition(0.1567);
                        bigWristR.setPosition(0.88);
                        bigWristL.setPosition(0.1194);
                        smallWrist.setPosition(0.05);
                    }
                })
                .addTemporalMarker(2.5,() -> {
                    extTarget = 1300; //1100
                })

                //placement
                .addTemporalMarker(3.4,() -> {
                    for(int i=0; i<100; i++) {
                        smallWrist.setPosition(0.17);
                    }
                })
                .addTemporalMarker(3.6, this::jerk)


                //to home position
                .addTemporalMarker(4.5,() -> {
                    extTarget = 0;
                })
                .addTemporalMarker(5.3,() -> {
                    for(int i=0; i<100; i++) {
                        spin.setPosition(0.1567);
                        bigWristR.setPosition(0.88);
                        bigWristL.setPosition(0.1194);
                        smallWrist.setPosition(0.1567);
                    }
                })
                .lineTo(new Vector2d(20, 5))
                //.addTemporalMarker(8.5,() -> {drive.followTrajectorySequenceAsync(dropOffII1, mail);})
                .build();
        //endregion

        //region DROP OFF II 1
        dropOffII1 = drive.trajectorySequenceBuilder(cycleI.end())
                //go to old wall position
                .lineTo(new Vector2d(15, -37))
                .waitSeconds(10)
                .addTemporalMarker(3,() -> {
                    controlState = poseControlState.OLDWALL;
                    extTarget = 40;
                    flpPosTarget = 1860;
                    smallWrist.setPosition(0.3206);
                })
                .addTemporalMarker(3.7,() -> {drive.followTrajectorySequenceAsync(dropOffII2, mail);})
                .build();
        //endregion

        //region DROP OFF II 2
        dropOffII2 = drive.trajectorySequenceBuilder(dropOffII1.end())
                .waitSeconds(1)
                .back(3)
                .waitSeconds(10)
                //far down
                .addTemporalMarker(0,() -> {
                    claw.setPosition(0.6);
                    spin.setPosition(0.7106);
                    for(int i=0; i<100; i++) {
                        bigWristR.setPosition(0.45);
                        bigWristL.setPosition(0.55);
                        smallWrist.setPosition(0.3206);
                    }
                })
                .addTemporalMarker(1,() -> {drive.followTrajectorySequenceAsync(dropOffII3, mail);})
                .build();
        //endregion

        //region DROP OFF II 3
        dropOffII3 = drive.trajectorySequenceBuilder(dropOffII2.end())
                .back(4)
                .waitSeconds(10)
                //close and lift
                .addTemporalMarker(0,() -> {
                    for(int i=0; i<100; i++) {
                        claw.setPosition(0.3);
                    }
                })
                .addTemporalMarker(1.5,() -> {
                    for(int i=0; i<100; i++) {
                        claw.setPosition(0.6);
                    }
                })
                .addTemporalMarker(2,() -> {
                    extTarget = 0;
                    for(int i=0; i<100; i++) {
                        bigWristR.setPosition(0.5394);
                        bigWristL.setPosition(0.46);
                    }
                })
                .addTemporalMarker(3.5,() -> {drive.followTrajectorySequenceAsync(cycleII, mail);})
                .build();
        //endregion


        //region CYCLE II
        cycleII = drive.trajectorySequenceBuilder(dropOffII3.end())
                .lineTo(new Vector2d(25, 17))
                .waitSeconds(0.1)
                .lineTo(new Vector2d(27, 17))
                .waitSeconds(0.3)

                //to high position
                .addTemporalMarker(0,() -> {
                    flpPosTarget = -100;
                })
                .addTemporalMarker(0,() -> {
                    for(int i=0; i<100; i++) {
                        spin.setPosition(0.1567);
                        bigWristR.setPosition(0.88);
                        bigWristL.setPosition(0.1194);
                        smallWrist.setPosition(0.05);
                    }
                })
                .addTemporalMarker(2.5,() -> {
                    extTarget = 1100;
                })

                //placement
                .addTemporalMarker(3.4,() -> {
                    for(int i=0; i<100; i++) {
                        smallWrist.setPosition(0.17);
                    }
                })
                .addTemporalMarker(3.6, this::jerk)


                //to home position
                .addTemporalMarker(4.5,() -> {
                    extTarget = 0;
                })
                .addTemporalMarker(5.3,() -> {
                    for(int i=0; i<100; i++) {
                        spin.setPosition(0.1567);
                        bigWristR.setPosition(0.88);
                        bigWristL.setPosition(0.1194);
                        smallWrist.setPosition(0.1567);
                    }
                })
                .lineTo(new Vector2d(20, 5))
                //.addTemporalMarker(8.5,() -> {drive.followTrajectorySequenceAsync(dropOffI, mail);})
                .build();
        //endregion

        drive.followTrajectorySequenceAsync(preload, mail);
        mail.setAutoEnd((new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading())));
    }

    public void miniPickup()
    {
        spin.setPosition(0.1572);
        smallWrist.setPosition(0.6828);
        bigWristR.setPosition(0.78);
        bigWristL.setPosition(0.22);
    }
    public void highPosition()
    {
        telemetry.addLine("HIGH POSITION");
        controlState = poseControlState.HIGH;
        if(flpPosTarget<200) {
            extTarget = 1060; //1160
            flpPosTarget = 0;
            spin.setPosition(0.1567);
            bigWristR.setPosition(0.88);
            bigWristL.setPosition(0.1194);
            smallWrist.setPosition(0.1567);
        }
        else {
            extTarget = 0;
            jerkTimer.reset();
            while(-extLMotor.getCurrentPosition()>10 || jerkTimer.time() < 1) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            flpPosTarget = 0;
            spin.setPosition(0.1567);
            bigWristR.setPosition(0.88);
            bigWristL.setPosition(0.1194);
            smallWrist.setPosition(0.1567);
            jerkTimer.reset();
            while(jerkTimer.time() < 1) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            extTarget = 1100;
            jerkTimer.reset();
            while(jerkTimer.time() < 0.5) {

                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
        }
    }
    public void homePosition()
    {
        telemetry.addLine("HOME POSITION");
        controlState = poseControlState.HOME;
        if(flpPosTarget<200) {
            extTarget = 0;
            flpPosTarget = 0;
            if(Math.abs(extLMotor.getCurrentPosition())>1200){
                spin.setPosition(0.1567);
                bigWristR.setPosition(0.3194);
                bigWristL.setPosition(0.68);
                smallWrist.setPosition(0.6372);
                while(jerkTimer.time() < 0.7) {
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
            }
            spin.setPosition(0.1567);
            bigWristR.setPosition(0.88);
            bigWristL.setPosition(0.1194);
            smallWrist.setPosition(0.1567);
        }
        else {
            extTarget = 0;
            jerkTimer.reset();
            while(jerkTimer.time() < 0.3 && Math.abs(extLMotor.getCurrentPosition())<50) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            spin.setPosition(0.1567);
            bigWristR.setPosition(0.88);
            bigWristL.setPosition(0.1194);
            smallWrist.setPosition(0.1567);
            jerkTimer.reset();
            while(jerkTimer.time() < 0.3) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            flpPosTarget = 0;
            jerkTimer.reset();
            while(jerkTimer.time() < 1) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
        }
    }
    public void oldWallPosition()
    {
        telemetry.addLine("OLD WALL POSITION");
        controlState = poseControlState.OLDWALL;
        if(flpPosTarget>1200) {
            extTarget = 40;
            flpPosTarget = 1860;
            if(Math.abs(extLMotor.getCurrentPosition())>1200){
                spin.setPosition(0.1567);
                bigWristR.setPosition(0.3194);
                bigWristL.setPosition(0.68);
                smallWrist.setPosition(0.6372);
                while(jerkTimer.time() < 0.7) {
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
            }
            spin.setPosition(0.1528);
            bigWristR.setPosition(0.48);
            bigWristL.setPosition(0.5183);
            smallWrist.setPosition(0.3206);
        }
        else {
            extTarget = 0;
            if(Math.abs(extLMotor.getCurrentPosition())>1200){
                spin.setPosition(0.1567);
                bigWristR.setPosition(0.3194);
                bigWristL.setPosition(0.68);
                smallWrist.setPosition(0.6372);
                while(jerkTimer.time() < 0.7) {
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
            }
            jerkTimer.reset();
            while(jerkTimer.time() < 1) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            spin.setPosition(0.1567);
            bigWristR.setPosition(0.88);
            bigWristL.setPosition(0.1194);
            smallWrist.setPosition(0.1567);
            flpPosTarget = 1860;
            jerkTimer.reset();
            while(jerkTimer.time() < 1.3) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            spin.setPosition(0.1528);
            bigWristR.setPosition(0.48);
            bigWristL.setPosition(0.5183);
            smallWrist.setPosition(0.3206);
            extTarget = 40;
            jerkTimer.reset();
            while(jerkTimer.time() < 0.3) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
        }
    }
    public void newWallPosition()
    {
        telemetry.addLine("NEW WALL POSITION");
        controlState = poseControlState.NEWWALL;
        if(flpPosTarget<200) {
            extTarget = 0;
            flpPosTarget = 0;
            jerkTimer.reset();
            while(jerkTimer.time() < 0.3) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            spin.setPosition(0.1567);
            bigWristR.setPosition(0.6094);
            bigWristL.setPosition(0.39);
            smallWrist.setPosition(0.5317);
            claw.setPosition(0.3);
        }
        else {
            extTarget = 0;
            jerkTimer.reset();
            while(jerkTimer.time() < 0.3) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            flpPosTarget = 0;
            spin.setPosition(0.1567);
            bigWristR.setPosition(0.88);
            bigWristL.setPosition(0.1194);
            smallWrist.setPosition(0.1567);
            jerkTimer.reset();
            while(jerkTimer.time() < 1) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            spin.setPosition(0.1567);
            bigWristR.setPosition(0.6094);
            bigWristL.setPosition(0.39);
            smallWrist.setPosition(0.5317);
            claw.setPosition(0.3);
        }
    }
    public void pickupPosition()
    {
        telemetry.addLine("PICKUP POSITION");
        controlState = poseControlState.PICKUP;
        if(flpPosTarget>1200) {
            extTarget = 40;
            flpPosTarget = 1860;
            spin.setPosition(0.1528);
            bigWristR.setPosition(0.39);
            bigWristL.setPosition(0.6078);
            smallWrist.setPosition(0.3206);
        }
        else {
            extTarget = 0;
            jerkTimer.reset();
            while(jerkTimer.time() < 0.3) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            spin.setPosition(0.1567);
            bigWristR.setPosition(0.88);
            bigWristL.setPosition(0.1194);
            smallWrist.setPosition(0.1567);
            flpPosTarget = 1860;
            jerkTimer.reset();
            while(jerkTimer.time() < 1) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            spin.setPosition(0.1528);
            bigWristR.setPosition(0.39);
            bigWristL.setPosition(0.6078);
            smallWrist.setPosition(0.3206);
            extTarget = 40;
            jerkTimer.reset();
            while(jerkTimer.time() < 0.3) {
                extOldCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
        }
    }
    public void jerk()
    {
        telemetry.addLine("JERK");
        extTarget = extTarget - 600;
        jerkTimer.reset();
        while (jerkTimer.time() < 1.4) {
            extOldCONTROLLER();
            flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
        }
        extTarget = 908;
        claw.setPosition(0.3);
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
        bigWristL.setPosition(0.05); //0.13
        bigWristR.setPosition(0.95); //0.87
        smallWrist.setPosition(0.1519); //0.1519
        claw.setPosition(0.6);
        spin.setPosition(0.1528);
    }
    public void movementInitII()
    {
        extTarget = 0;
        bigWristL.setPosition(0.13);
        bigWristR.setPosition(0.87);
        smallWrist.setPosition(0.1519);
        claw.setPosition(0.6);
        spin.setPosition(0.1528);
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
        if(velocityVal>1500){
            velocityVal=1500;
        }
        else if (velocityVal<-1500)
        {
            velocityVal = -1500;
        }
        flipMotor.setVelocity(velocityVal);
    }
}