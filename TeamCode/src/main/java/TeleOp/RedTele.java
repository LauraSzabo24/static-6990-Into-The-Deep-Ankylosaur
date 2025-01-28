package TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;
import Autonomous.Mailbox;

@TeleOp
@Config
public class RedTele extends LinearOpMode {
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
    FtcDashboard dashboard;
    //endregion

    //region DRIVER A MATERIAL
    NewMecanumDrive drive;
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

    //INITIALIZATIONS
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
        drive.reverseMotors();
        drive.setPoseEstimate(Mailbox.currentPose);

        //servos
        smallWrist = hardwareMap.get(Servo.class, "SW");
        bigWristL = hardwareMap.get(Servo.class, "BWL");
        bigWristR = hardwareMap.get(Servo.class, "BWR");
        spin = hardwareMap.get(Servo.class, "SPIN");
        claw = hardwareMap.get(Servo.class, "CLAW");

        //gamepads
        currG1 = new Gamepad();
        oldG1 = new Gamepad();
        currG2 = new Gamepad();
        oldG2 = new Gamepad();

        //mailbox
        imu = hardwareMap.get(IMU.class, "imu");
        parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }
    public void movementInitI()
    {
        extTarget = 0;
        notNormalLimits = false;
        bigWristL.setPosition(0.1967);
        bigWristR.setPosition(0.8289);
        smallWrist.setPosition(0.1967);
        claw.setPosition(0.7);
        spin.setPosition(0.725);
    }
    public void movementInitII()
    {
        extTarget = 0;
        bigWristL.setPosition(0.13);
        bigWristR.setPosition(0.87);
        smallWrist.setPosition(0.1519);
        claw.setPosition(0.7);
        spin.setPosition(0.1528);
    }

    //BASICS
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardwareInit();
        flpLowLimit = 0;
        flpHighLimit = 3500;

        movementInitI();
        clawIH = true;
        controlState = poseControlState.FREE;
        gameModeA = speedControlState.NORMAL;
        gameModeB = speedControlState.NORMAL;
        hangState = hangControlState.PHASEONE;
        editMode = true;
        waitForStart();

        //movementInitII();
        double cumutime = 0;
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            mainLoop();
        }
    }
    public void mainLoop()
    {
        //UPDATES
        oldG1.copy(currG1);
        oldG2.copy(currG2);
        currG1.copy(gamepad1);
        currG2.copy(gamepad2);
        telemetry.update();
        drive.update();
        poseEstimate = new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading());

        //CONTROLS
        driverAControls();
        driverBControls();
        telemetry.addData("CURRENT POSITION STATE", controlState);
        stateCheck();

        //EXTENDER & FLIPPER
        extOldCONTROLLER();
        flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
    }

    //DRIVING CONTROLS
    public void driverAControls()
    {
        //region SPEED CHANGES | LEFT TRIGGER FAST | RIGHT TRIGGER SLOW
        gameModeA = speedControlState.NORMAL;
        if(flashTimer.time()>2) {
            gamepad1.setLedColor(0, 0, (255 / 255.0), Gamepad.LED_DURATION_CONTINUOUS);
        }
        if (gamepad1.right_trigger > 0.3) {
            gameModeA = speedControlState.PRECISION;
            gamepad1.setLedColor((38/255.0), (255/255.0), 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
        if (gamepad1.left_trigger > 0.3) {
            gameModeA = speedControlState.SUPERSPEED;
            gamepad1.setLedColor((255/255.0), 0, (125/255.0), Gamepad.LED_DURATION_CONTINUOUS);
        }
        switch(gameModeA){
            case NORMAL:
                multiply = 1;
                speed = 3;
                break;
            case PRECISION:
                multiply = 0.5;
                speed = 3;
                break;
            case SUPERSPEED:
                multiply =0.7;
                speed = 1;
                break;
        }
        //endregion

        //region FIELD CENTRIC RESET
        if(!oldG1.a && !oldG2.b && !oldG2.x && !oldG2.y && ((currG1.a && currG1.b) || (currG1.a && currG1.y) || (currG1.a && currG1.x) || (currG1.b && currG1.y) || (currG1.b && currG1.x) || (currG1.x && currG1.y)))
        {
            drive.resetOdo();
            gamepad1.runLedEffect(resetField);
            flashTimer.reset();
        }
        //endregion

        //region FIELD CENTRIC
        poseEstimate = drive.getPoseEstimate();
        Vector2d input = new Vector2d(
                ((gamepad1.left_stick_y)* multiply)/speed,
                ((gamepad1.left_stick_x)* multiply)/speed
        ).rotated(-poseEstimate.getHeading()); // -odo.getHeading() -poseEstimate.getHeading()
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        ((gamepad1.right_stick_x * multiply)/speed)
                )
        );
        drive.update();
        //endregion

    }
    public void driverBControls()
    {
        //region TRANSITION
        if(gamepad2.right_bumper)
        {
            editMode = false;
            gameModeB = speedControlState.NORMAL;
            notNormalLimits = false;
            gamepad2.setLedColor((38/255.0), (255/255.0), 0, Gamepad.LED_DURATION_CONTINUOUS);

        }
        else {
            editMode = true;
            gamepad2.setLedColor((255/255.0), 0, (125/255.0), Gamepad.LED_DURATION_CONTINUOUS);
        }
        //endregion

        if(editMode)
        {
            //region SPEED CONTROLS | LEFT TRIGGER FAST | RIGHT TRIGGER SLOW
            int extAmount = 80;
            double smallAmount = 0.01;
            double bigAmount = 0.03;
            gameModeB = speedControlState.NORMAL;
            if (gamepad2.right_trigger > 0.3) {
                gameModeB = speedControlState.PRECISION;
            }
            if (gamepad2.left_trigger > 0.3) {
                gameModeB = speedControlState.SUPERSPEED;
            }
            switch(gameModeB){
                case NORMAL:
                    break;
                case PRECISION:
                    extAmount = 40;
                    smallAmount = 0.004;
                    bigAmount = 0.01;
                    break;
                case SUPERSPEED:
                    extAmount = 120;
                    smallAmount = 0.03;
                    bigAmount = 0.06;
                    break;
            }
            //endregion

            //region BIG WRIST Y/GREEN->UP  A/BLUE->DOWN
            if(gamepad2.y && bigWristR.getPosition()>=0)
            {
                telemetry.addLine("WRIST MOVEMENT");
                controlState = poseControlState.FREE;
                variablePickup = false;
                bigWristL.setPosition(bigWristL.getPosition() + bigAmount);
                bigWristR.setPosition(bigWristR.getPosition() - bigAmount);
            }
            else if(gamepad2.a && bigWristR.getPosition()<=0.8094+bigAmount)
            {
                telemetry.addLine("WRIST MOVEMENT");
                controlState = poseControlState.FREE;
                variablePickup = false;
                bigWristL.setPosition(bigWristL.getPosition() - bigAmount);
                bigWristR.setPosition(bigWristR.getPosition() + bigAmount);
            }
            else if (gamepad2.x) //OLD WALL  X/PINK
            {
                spin.setPosition(0.7122);
                smallWrist.setPosition(0.5667);
                bigWristR.setPosition(0.8094);
                bigWristL.setPosition(0.1878);
            }
            else if(gamepad2.b){ //NEW WALL  B?RED
                spin.setPosition(0.7111);
                smallWrist.setPosition(0.4867);
                bigWristL.setPosition(0.7594);
                bigWristR.setPosition(0.24);
            }
            //endregion

            //region SMALL WRIST
            if(gamepad2.left_stick_x>0 && smallWrist.getPosition()<(0.96-smallAmount))
            {
                telemetry.addLine("WRIST MOVEMENT");
                smallWrist.setPosition(smallWrist.getPosition() + smallAmount);
                controlState = poseControlState.FREE;
                variablePickup = false;
            }
            else if(gamepad2.left_stick_x<0 && smallWrist.getPosition()>=0.1367-smallAmount)
            {
                telemetry.addLine("WRIST MOVEMENT");
                smallWrist.setPosition(smallWrist.getPosition() - smallAmount);
                controlState = poseControlState.FREE;
                variablePickup = false;
            }
            if(currG2.left_stick_button && !oldG2.left_stick_button)
            {
                telemetry.addLine("WRIST MOVEMENT");
                controlState = poseControlState.FREE;
                variablePickup = false;
                smallWristPos ++;
                if(smallWristPos>2)
                {
                    smallWristPos = 0;
                }
                switch(smallWristPos)
                {
                    case 0:
                        smallWrist.setPosition(0.1367);
                        break;
                    case 1:
                        smallWrist.setPosition(0.5667);
                        break;
                    case 2:
                        smallWrist.setPosition(0.96);
                        break;
                }
            }
            //endregion

            //region SPINNER
            if(gamepad2.right_stick_x>0 || gamepad2.right_stick_x<0)
            {
                telemetry.addLine("SPINNER MOVEMENT");
                spin.setPosition(spin.getPosition() + (gamepad2.right_stick_x * 0.05));
                controlState = poseControlState.FREE;
            }
            if(currG2.right_stick_button && !oldG2.right_stick_button)
            {
                spinnerPos ++;
                if(spinnerPos>3)
                {
                    spinnerPos = 0;
                }
                switch(spinnerPos)
                {
                    case 0:
                        spin.setPosition(0.1528);
                        break;
                    case 1:
                        spin.setPosition(0.4239);
                        break;
                    case 2:
                        spin.setPosition(0.7111);
                        break;
                    case 3:
                        spin.setPosition(1);
                        break;
                }
            }
            //endregion

            //region EXTENDER
            if(gamepad2.dpad_up && extTarget<=1520)
            {
                telemetry.addLine("ext UP");
                controlState = poseControlState.FREE;
                if(flpPosTarget<200)
                {
                    if (extTarget + extAmount >= 1520 - extAmount) {
                        extTarget += Math.abs(Math.abs(extTarget) - 1520);
                    } else {
                        extTarget += extAmount;
                    }
                }
                else if(flpPosTarget>=1600)
                {
                    if(extTarget<=700) {
                        if (extTarget + extAmount >= 700 - extAmount) {
                            extTarget += Math.abs(Math.abs(extTarget) - 700);
                        } else {
                            extTarget += extAmount;
                        }
                    }
                }
            }
            else if(gamepad2.dpad_down && extTarget>=0)
            {
                telemetry.addLine("ext DOWN");
                controlState = poseControlState.FREE;
                if(flpPosTarget<200) {
                    if (extTarget - extAmount <= 0) {
                        extTarget -= Math.abs(extTarget);
                    } else {
                        extTarget -= extAmount;
                    }
                }
                else if(flpPosTarget>=1600)
                {
                    if (extTarget - extAmount <= extAmount) {
                        extTarget -= Math.abs(extTarget-extAmount);
                    } else {
                        extTarget -= extAmount;
                    }
                }
            }
            //endregion

            //region VARIABLE PICKUP
            if(variablePickup)
            {
                extPercentage = (extTarget-40.0)/700;
                spin.setPosition(0.7472);
                smallWrist.setPosition(0.1744 + ((0.245-0.1744)*extPercentage));
                bigWristL.setPosition(0.4189 + ((0.39-0.4189)*extPercentage));
                bigWristR.setPosition(0.58 + ((0.6089-0.58)*extPercentage));

                //EXT          40         700
                //smallWrist - 0.1744 - 0.245
                //bigWristL - 0.4189 - 0.39
                //bigWristR - 0.58- 0.6089
            }
            //endregion

            //region FLIPPER
            if(gamepad2.dpad_left && flpPosTarget>=flpLowLimit)
            {
                telemetry.addLine("flp UP");
                controlState = poseControlState.FREE;
                variablePickup = false;
                if(flpPosTarget-20<=0)
                {
                    flpPosTarget-=Math.abs(flpPosTarget);
                }
                else {
                    flpPosTarget-=20;
                }
            }
            else if(gamepad2.dpad_right && flpPosTarget<flpHighLimit && !(extTarget>500 && flpPosTarget<2500 && flpHighLimit==3500))
            {
                telemetry.addLine("flp DOWN");
                controlState = poseControlState.FREE;
                variablePickup = false;
                if(flpPosTarget+20>=flpHighLimit)
                {
                    flpPosTarget+=Math.abs(flpHighLimit-flpPosTarget);
                }
                else {
                    flpPosTarget+=20;
                }
            }

            //COMBO MOVEMENT EXTENSION
           /* if(extTarget>=200)
            {
                double flpPercentage = (flpPosTarget/1620.0);
                extTarget -= extTarget*flpPercentage;
            }*/
            //endregion

            //region CLAW
            if (currG2.touchpad && !oldG2.touchpad)
            {
                if(clawIH)
                {
                    claw.setPosition(0.7);
                }
                else {
                    claw.setPosition(0.3);
                }
                clawIH = !clawIH;
            }
            //endregion

            //region LIMIT REMOVER
            if(currG2.left_bumper && !oldG2.left_bumper && flpPosTarget>=flpLowLimit)
            {
                flpHighLimit = Integer.MAX_VALUE;
                flpLowLimit = -Integer.MAX_VALUE;
            }
            //endregion
        }
        else {
            setStates();
        }
    }
    public void setStates()
    {
        //region HIGH POSITION  Y/GREEN
        if(controlState != poseControlState.HIGH && currG2.y && !oldG2.y)
        {
            telemetry.addLine("HIGH POSITION");
            controlState = poseControlState.HIGH;
            variablePickup = false;
            if(flpPosTarget<1000) {
                flpPosTarget = 0;
                if(Math.abs(extLMotor.getCurrentPosition())>1200 || bigWristR.getPosition()>0.6){
                    spin.setPosition(0.7106);
                    bigWristR.setPosition(0.88);
                    bigWristL.setPosition(0.1194);
                    smallWrist.setPosition(0.1567);
                    while(jerkTimer.time() < 1.2) {
                        driverAControls();
                        extOldCONTROLLER();
                        flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                    }
                }
                extTarget = 1240;
                spin.setPosition(0.7111);
                bigWristR.setPosition(0.0389);
                bigWristL.setPosition(0.9589);
                smallWrist.setPosition(0.175);
            }
            else {
                extTarget = 0;
                jerkTimer.reset();
                while(-extLMotor.getCurrentPosition()>50 || jerkTimer.time() < 1) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                spin.setPosition(0.7111);
                smallWrist.setPosition(0.1367);
                bigWristL.setPosition(1);
                bigWristR.setPosition(0);
                jerkTimer.reset();
                while(jerkTimer.time() < 1.3) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                flpPosTarget = 0;
                jerkTimer.reset();
                while(jerkTimer.time() < 1.8) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                spin.setPosition(0.7111);
                bigWristR.setPosition(0.0389);
                bigWristL.setPosition(0.9589);
                smallWrist.setPosition(0.175);
                extTarget = 1240;
                jerkTimer.reset();
                while(jerkTimer.time() < 0.5) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
            }
        }
        //endregion

        //region NEW WALL POSITION  B/RED
        if(controlState != poseControlState.NEWWALL && currG2.b && !oldG2.b)
        {
            telemetry.addLine("NEW WALL POSITION");
            controlState = poseControlState.NEWWALL;
            variablePickup = false;
            if(flpPosTarget<1000) {
                flpPosTarget = 0;
                if(Math.abs(extLMotor.getCurrentPosition())>1200 || bigWristR.getPosition()>0.6){
                    spin.setPosition(0.1567);
                    bigWristR.setPosition(0.88);
                    bigWristL.setPosition(0.1194);
                    smallWrist.setPosition(0.1567);
                    while(jerkTimer.time() < 1.2) {
                        driverAControls();
                        extOldCONTROLLER();
                        flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                    }
                }
                extTarget = 280;
                spin.setPosition(0.1567);
                bigWristR.setPosition(0.06);
                bigWristL.setPosition(0.94);
                smallWrist.setPosition(0.215);
            }
            else {
                extTarget = 0;
                jerkTimer.reset();
                while(-extLMotor.getCurrentPosition()>50 || jerkTimer.time() < 1) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                spin.setPosition(0.7111);
                smallWrist.setPosition(0.1367);
                bigWristL.setPosition(1);
                bigWristR.setPosition(0);
                jerkTimer.reset();
                while(jerkTimer.time() < 1.3) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                flpPosTarget = 0;
                jerkTimer.reset();
                while(jerkTimer.time() < 1.8) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                extTarget = 280;
                spin.setPosition(0.1567);
                bigWristR.setPosition(0.06);
                bigWristL.setPosition(0.94);
                smallWrist.setPosition(0.215);
            }
        }
        //endregion

        //region OLD WALL POSITION  X/PINK
        if(controlState != poseControlState.OLDWALL && ((currG2.x && !oldG2.x)))
        {
            telemetry.addLine("OLD WALL POSITION");
            controlState = poseControlState.OLDWALL;
            variablePickup = false;
            if(flpPosTarget<700) {
                extTarget = 0;
                flpPosTarget = 0;
                if(Math.abs(extLMotor.getCurrentPosition())>1200){
                    spin.setPosition(0.7111);
                    smallWrist.setPosition(0.1367);
                    bigWristL.setPosition(1);
                    bigWristR.setPosition(0);
                    while(jerkTimer.time() < 1.2) {
                        driverAControls();
                        extOldCONTROLLER();
                        flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                    }
                }
                jerkTimer.reset();
                while(jerkTimer.time() < 0.3) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                spin.setPosition(0.7122);
                smallWrist.setPosition(0.5667);
                bigWristR.setPosition(0.8094);
                bigWristL.setPosition(0.1878);
            }
            else {
                extTarget = 0;
                jerkTimer.reset();
                while(jerkTimer.time() < 0.3) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                flpPosTarget = 0;
                spin.setPosition(0.7111);
                smallWrist.setPosition(0.1367);
                bigWristL.setPosition(1);
                bigWristR.setPosition(0);
                jerkTimer.reset();
                while(jerkTimer.time() < 1) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                spin.setPosition(0.7122);
                smallWrist.setPosition(0.5667);
                bigWristR.setPosition(0.8094);
                bigWristL.setPosition(0.1878);
            }
        }
        //endregion

        //region PICKUP POSITION  A/BLUE
        if(controlState != poseControlState.PICKUP && currG2.a && !oldG2.a)
        {
            telemetry.addLine("PICKUP POSITION");
            controlState = poseControlState.PICKUP;
            variablePickup = false;
            if(flpPosTarget>3200) {
                extTarget = 40;
                flpPosTarget = 3500;
                if(Math.abs(extLMotor.getCurrentPosition())>1200){
                    while(extLMotor.getCurrentPosition()>100 && jerkTimer.time() < 0.7) {
                        driverAControls();
                        extOldCONTROLLER();
                        flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                    }
                }
                spin.setPosition(0.7472);
                bigWristR.setPosition(0.52);
                bigWristL.setPosition(0.4789);
                smallWrist.setPosition(0.1294);
            }
            else {
                extTarget = 0;
                spin.setPosition(0.7106);
                bigWristR.setPosition(0);
                bigWristL.setPosition(1);
                smallWrist.setPosition(0.1367);
                jerkTimer.reset();
                while(extLMotor.getCurrentPosition()>1200 && jerkTimer.time() < 1) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                flpPosTarget = 3500;
                jerkTimer.reset();
                while(jerkTimer.time() < 1.8) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                spin.setPosition(0.7472);
                bigWristR.setPosition(0.52);
                bigWristL.setPosition(0.4789);
                smallWrist.setPosition(0.1294);
                extTarget = 40;
            }
        }
        else if (currG2.a && !oldG2.a)
        {
            telemetry.addLine("VARIABLE PICKUP");
            controlState = poseControlState.VARIPICKUP;
            variablePickup = true;
            claw.setPosition(0.3);
        }
        if(!(controlState == poseControlState.VARIPICKUP || controlState == poseControlState.FREE))
        {
            variablePickup = false;
        }
        //endregion

        //region HIGH BASKET  D-UP
        if(controlState != poseControlState.HIGHBASKET && currG2.dpad_up && !oldG2.dpad_up)
        {
            telemetry.addLine("HIGH BASKET");
            controlState = poseControlState.HIGHBASKET;
            variablePickup = false;
            if(flpPosTarget<1000) {
                flpPosTarget = 0;
                if(Math.abs(extLMotor.getCurrentPosition())>1200 || bigWristR.getPosition()>0.6){
                    spin.setPosition(0.4039);
                    bigWristR.setPosition(0.88);
                    bigWristL.setPosition(0.1194);
                    smallWrist.setPosition(0.1567);
                    while(jerkTimer.time() < 1.2) {
                        driverAControls();
                        extOldCONTROLLER();
                        flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                    }
                }
                extTarget = 1520;
                spin.setPosition(0.4039);
                bigWristR.setPosition(0.5283);
                bigWristL.setPosition(0.4672);
                smallWrist.setPosition(0.96);
            }
            else {
                extTarget = 0;
                jerkTimer.reset();
                while(-extLMotor.getCurrentPosition()>50 || jerkTimer.time() < 1) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                spin.setPosition(0.4039);
                smallWrist.setPosition(0.1367);
                bigWristL.setPosition(1);
                bigWristR.setPosition(0);
                jerkTimer.reset();
                while(jerkTimer.time() < 1.3) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                flpPosTarget = 0;
                jerkTimer.reset();
                while(jerkTimer.time() < 1.8) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                spin.setPosition(0.4039);
                bigWristR.setPosition(0.5283);
                bigWristL.setPosition(0.4672);
                smallWrist.setPosition(0.96);
                extTarget = 1520;
                jerkTimer.reset();
                while(jerkTimer.time() < 0.5) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
            }
        }
        //endregion

        //region LOW BASKET  D-LEFT
        if(controlState != poseControlState.LOWBASKET && currG2.dpad_left && !oldG2.dpad_left)
        {
            telemetry.addLine("LOW BASKET");
            controlState = poseControlState.LOWBASKET;
            variablePickup = false;
            if(flpPosTarget<1000) {
                flpPosTarget = 0;
                if(Math.abs(extLMotor.getCurrentPosition())>1200 || bigWristR.getPosition()>0.6){
                    spin.setPosition(0.4039);
                    bigWristR.setPosition(0.88);
                    bigWristL.setPosition(0.1194);
                    smallWrist.setPosition(0.1567);
                    while(jerkTimer.time() < 1.2) {
                        driverAControls();
                        extOldCONTROLLER();
                        flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                    }
                }
                extTarget = 560;
                spin.setPosition(0.4039);
                bigWristR.setPosition(0.5283);
                bigWristL.setPosition(0.4672);
                smallWrist.setPosition(0.96);
            }
            else {
                extTarget = 0;
                jerkTimer.reset();
                while(-extLMotor.getCurrentPosition()>50 || jerkTimer.time() < 1) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                spin.setPosition(0.4039);
                smallWrist.setPosition(0.1367);
                bigWristL.setPosition(1);
                bigWristR.setPosition(0);
                jerkTimer.reset();
                while(jerkTimer.time() < 1.3) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                flpPosTarget = 0;
                jerkTimer.reset();
                while(jerkTimer.time() < 1.8) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                spin.setPosition(0.4039);
                bigWristR.setPosition(0.5283);
                bigWristL.setPosition(0.4672);
                smallWrist.setPosition(0.96);
                extTarget = 560;
                jerkTimer.reset();
                while(jerkTimer.time() < 0.5) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
            }
        }
        //endregion

        //region LOW POSITION  D-DOWN
        if(controlState != poseControlState.LOW && currG2.dpad_down && !oldG2.dpad_down)
        {
            telemetry.addLine("LOW POSITION");
            controlState = poseControlState.LOW;
            variablePickup = false;
            if(flpPosTarget<1000) {
                flpPosTarget = 0;
                if(Math.abs(extLMotor.getCurrentPosition())>1200 || bigWristR.getPosition()>0.8){
                    spin.setPosition(0.7106);
                    bigWristR.setPosition(0.88);
                    bigWristL.setPosition(0.1194);
                    smallWrist.setPosition(0.1567);
                    while(jerkTimer.time() < 1.2) {
                        driverAControls();
                        extOldCONTROLLER();
                        flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                    }
                }
                extTarget = 440;
                spin.setPosition(0.7111);
                bigWristR.setPosition(0.0389);
                bigWristL.setPosition(0.9589);
                smallWrist.setPosition(0.175);
            }
            else {
                extTarget = 0;
                jerkTimer.reset();
                while(jerkTimer.time() < 0.3) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                spin.setPosition(0.7111);
                smallWrist.setPosition(0.1367);
                bigWristL.setPosition(1);
                bigWristR.setPosition(0);
                jerkTimer.reset();
                while(jerkTimer.time() < 1.3) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                flpPosTarget = 0;
                jerkTimer.reset();
                while(jerkTimer.time() < 1.5) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                spin.setPosition(0.7111);
                bigWristR.setPosition(0.0389);
                bigWristL.setPosition(0.9589);
                smallWrist.setPosition(0.175);
                extTarget = 440;
                jerkTimer.reset();
                while(jerkTimer.time() < 0.5) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
            }
        }
        //endregion

        //region HOME POSITION  D-RIGHT
        if(controlState != poseControlState.HOME && ((currG2.dpad_right && !oldG2.dpad_right)))
        {
            telemetry.addLine("HOME POSITION");
            controlState = poseControlState.HOME;
            variablePickup = false;
            if(flpPosTarget<700) {
                extTarget = 0;
                flpPosTarget = 0;
                if(Math.abs(extLMotor.getCurrentPosition())>1200){
                    spin.setPosition(0.7111);
                    bigWristR.setPosition(0.3194);
                    bigWristL.setPosition(0.68);
                    smallWrist.setPosition(0.6372);
                    while(jerkTimer.time() < 1.2) {
                        driverAControls();
                        extOldCONTROLLER();
                        flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                    }
                }
                spin.setPosition(0.7111);
                smallWrist.setPosition(0.1367);
                bigWristL.setPosition(0.95);
                bigWristR.setPosition(0.05);
            }
            else {
                extTarget = 0;
                jerkTimer.reset();
                while(jerkTimer.time() < 0.3 && Math.abs(extLMotor.getCurrentPosition())<50) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                spin.setPosition(0.7111);
                smallWrist.setPosition(0.1367);
                bigWristL.setPosition(0.95);
                bigWristR.setPosition(0.05);
                jerkTimer.reset();
                while(jerkTimer.time() < 0.3) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                flpPosTarget = 0;
                jerkTimer.reset();
                while(jerkTimer.time() < 1) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
            }
        }
        //endregion

        //region JERK  L-BUMPER
        if( flipMotor.getCurrentPosition()>-2000 && currG2.left_bumper && !oldG2.left_bumper)
        {
            telemetry.addLine("JERK");
            variablePickup = false;
            if(extTarget>=300) {
                extTarget = extTarget + 700;
                jerkTimer.reset();
                while (jerkTimer.time() < 1.6) {
                    driverAControls();
                    extOldCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
            }
            claw.setPosition(0.3);
        }
        //endregion

        //region CLAW  TOUCH
        if (currG2.touchpad && !oldG2.touchpad)
        {
            if(clawIH)
            {
                claw.setPosition(0.7);
            }
            else {
                claw.setPosition(0.3);
            }
            clawIH = !clawIH;
        }
        //endregion
    }

    //CONTROLLERS
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

    //RANDOM
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
    public void stateCheck()
    {
        if(clawIH)
        {
            telemetry.addLine("CLAW CLOSE");
        }
        else{
            telemetry.addLine("CLAW OPEN");
        }

        telemetry.addData("GAMEMODEB", gameModeB);
        telemetry.addData("flpPosition ", flipMotor.getCurrentPosition());
        telemetry.addData("flpTarget", flpPosTarget);
        telemetry.addData("ext TARGET - ", extTarget);
        telemetry.addData("extPosition ", -extLMotor.getCurrentPosition());
        telemetry.addData("extVelocity ", -extLMotor.getVelocity());
        telemetry.addData("\next percent - ", extPercentage);

        telemetry.addData("\nspinner - ", spin.getPosition());
        telemetry.addData("small wrist - ", smallWrist.getPosition());
        telemetry.addData("big wrist right - ", bigWristR.getPosition());
        telemetry.addData("big wrist left - ", bigWristL.getPosition());
        telemetry.addData("claw - ", claw.getPosition());

        telemetry.addData("X ODOMETRY POSITION", poseEstimate.getX());
        telemetry.addData("Y ODOMETRY POSITION", poseEstimate.getY());
        telemetry.addData("HEADING ODOMETRY POSITION", poseEstimate.getHeading());
    }
}