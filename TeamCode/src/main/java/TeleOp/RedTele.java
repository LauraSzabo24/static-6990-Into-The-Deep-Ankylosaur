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
    boolean hangPhaseOne = true;
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
    speedControlState gameModeA;
    speedControlState gameModeB;
    poseControlState currentState;
    boolean editMode = true;
    boolean notNormalLimits = false;
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

        //odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(-3, -26);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();
    }
    public void movementInitI()
    {
        extTarget = 0;
        notNormalLimits = false;
        bigWristL.setPosition(0.13);
        bigWristR.setPosition(0.87);
        smallWrist.setPosition(0.1519);
        claw.setPosition(0.6);
        spin.setPosition(0.1528);
    }
    public void movementInitII()
    {
    }

    //BASICS
    @Override
    public void runOpMode() throws InterruptedException
    {
        hardwareInit();

        movementInitI();
        clawIH = true;
        currentState = poseControlState.FREE;
        gameModeA = speedControlState.NORMAL;
        gameModeB = speedControlState.NORMAL;
        editMode = true;
        waitForStart();

        movementInitII();
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
        //imu.initialize(parameters);
        odo.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
        //poseEstimate = new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading());

        //CONTROLS
        driverAControls();
        driverBControls();
        telemetry.addData("CURRENT POSITION STATE", currentState);
        stateCheck();

        //EXTENDER & FLIPPER
        extCONTROLLER();
        flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
    }

    //DRIVING CONTROLS
    public void driverAControls()
    {
        //region SPEED CHANGES | LEFT TRIGGER FAST | RIGHT TRIGGER SLOW
        if (gamepad1.right_trigger > 0) {
            gameModeA = speedControlState.PRECISION;
        }
        if (gamepad1.left_trigger > 0) {
            gameModeA = speedControlState.SUPERSPEED;
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
        if((currG1.a && currG1.b) || (currG1.a && currG1.y) || (currG1.a && currG1.x) || (currG1.b && currG1.y) || (currG1.b && currG1.x) || (currG1.x && currG1.y))
        {
            odo.recalibrateIMU();
            gamepad1.runLedEffect(resetField);
        }
        //endregion

        //region FIELD CENTRIC
        //poseEstimate = drive.getPoseEstimate();
        Vector2d input = new Vector2d(
                -((gamepad1.left_stick_y)* multiply)/speed,
                -((gamepad1.left_stick_x)* multiply)/speed
        ).rotated(-odo.getHeading()); // -odo.getHeading() -poseEstimate.getHeading()
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        ((gamepad1.right_stick_x * multiply)/speed)
                )
        );
        drive.update();
        //endregion

        //region HANG
        if(currG1.right_bumper && currG1.left_bumper)
        {
            if(hangPhaseOne)
            {
                telemetry.addLine("HANG PHASE ONE");
                gamepad2.setLedColor(0, (255/255.0), (149/255.0), Gamepad.LED_DURATION_CONTINUOUS);
                if(flpPosTarget<200) {
                    extTarget = 1160;
                    flpPosTarget = 0;
                    spin.setPosition(0.1567);
                    bigWristR.setPosition(0.88);
                    bigWristL.setPosition(0.1194);
                    smallWrist.setPosition(0.1567);
                }
                else {
                    extTarget = 0;
                    jerkTimer.reset();
                    while(jerkTimer.time() < 0.3) {
                        driverAControls();
                        extCONTROLLER();
                        flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                    }
                    flpPosTarget = 0;
                    while(jerkTimer.time() < 1) {
                        driverAControls();
                        extCONTROLLER();
                        flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                    }
                    spin.setPosition(0.1567);
                    bigWristR.setPosition(0.88);
                    bigWristL.setPosition(0.1194);
                    smallWrist.setPosition(0.1567);
                    extTarget = 1160;
                    while(jerkTimer.time() < 0.5) {
                        driverAControls();
                        extCONTROLLER();
                        flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                    }
                }
            }
            else{
                telemetry.addLine("HANG PHASE TWO");
                gamepad2.setLedColor((183/255.0), 0, (255/255.0), Gamepad.LED_DURATION_CONTINUOUS);
                extTarget = 0;
                while(opModeIsActive())
                {
                    extCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                    RAINBOW(0.005);
                }
            }
        }
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
            //region SPEED CONTROLS
            if ((currG2.right_trigger > 0.3) && !(oldG2.right_trigger>0.3) && (!notNormalLimits)) {
                gameModeB = speedControlState.PRECISION;
                notNormalLimits = true;
            }
            else if((currG2.right_trigger > 0.3) && !(oldG2.right_trigger>0.3))
            {
                notNormalLimits = false;
                gameModeB = speedControlState.NORMAL;
            }

            if ((currG2.left_trigger > 0.3) && !(oldG2.left_trigger>0.3) && (!notNormalLimits))  {
                gameModeB = speedControlState.SUPERSPEED;
                notNormalLimits = true;
            }
            else if((currG2.left_trigger > 0.3) && !(oldG2.left_trigger>0.3))
            {
                notNormalLimits = false;
                gameModeB = speedControlState.NORMAL;
            }
            //endregion

            //region BIG WRIST
            if(gamepad2.b && bigWristR.getPosition()>=0)
            {
                telemetry.addLine("WRIST MOVEMENT");
                currentState = poseControlState.FREE;
                bigWristL.setPosition(bigWristL.getPosition() + 0.01);
                bigWristR.setPosition(bigWristR.getPosition() - 0.01);
            }
            else if(gamepad2.x && bigWristR.getPosition()<=0.87+0.01)
            {
                telemetry.addLine("WRIST MOVEMENT");
                currentState = poseControlState.FREE;
                bigWristL.setPosition(bigWristL.getPosition() - 0.01);
                bigWristR.setPosition(bigWristR.getPosition() + 0.01);
            }
            else if (gamepad2.y)
            {
                bigWristL.setPosition(0.7294);
                bigWristR.setPosition(0.27);
            }
            else if(gamepad2.a){
                bigWristL.setPosition(0.13);
                bigWristR.setPosition(0.87);
            }
            //endregion

            //region SMALL WRIST
            if(gamepad2.left_stick_x>0 && smallWrist.getPosition()<(0.9367-0.005))
            {
                telemetry.addLine("WRIST MOVEMENT");
                smallWrist.setPosition(smallWrist.getPosition() + 0.005);
                currentState = poseControlState.FREE;
            }
            else if(gamepad2.left_stick_x<0 && smallWrist.getPosition()>=0.1519-0.005)
            {
                telemetry.addLine("WRIST MOVEMENT");
                smallWrist.setPosition(smallWrist.getPosition() - 0.005);
                currentState = poseControlState.FREE;
            }
            if(currG2.left_stick_button && !oldG2.left_stick_button)
            {
                telemetry.addLine("WRIST MOVEMENT");
                currentState = poseControlState.FREE;
                smallWristPos ++;
                if(smallWristPos>2)
                {
                    smallWristPos = 0;
                }
                switch(smallWristPos)
                {
                    case 0:
                        smallWrist.setPosition(0.1467);
                        break;
                    case 1:
                        smallWrist.setPosition(0.5467);
                        break;
                    case 2:
                        smallWrist.setPosition(0.9367);
                        break;
                }
            }
            //endregion

            //region SPINNER
            if(gamepad2.right_stick_x>0 || gamepad2.right_stick_x<0)
            {
                telemetry.addLine("SPINNER MOVEMENT");
                spin.setPosition(spin.getPosition() + (gamepad2.right_stick_x * 0.05));
                currentState = poseControlState.FREE;
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
            if(gamepad2.dpad_up && extTarget<=1500)
            {
                telemetry.addLine("ext UP");
                currentState = poseControlState.FREE;
                if(extTarget+40>=1500-40)
                {
                    extTarget+=Math.abs(Math.abs(extTarget)-1500);
                }
                else {
                    extTarget+=40;
                }
            }
            else if(gamepad2.dpad_down && extTarget>=0)
            {
                telemetry.addLine("ext DOWN");
                currentState = poseControlState.FREE;
                if(extTarget-40<=0)
                {
                    extTarget-=Math.abs(extTarget);
                }
                else {
                    extTarget-=40;
                }
            }
            //endregion

            //region COMBO MOVEMENT PICKUP
            /*if(flpPosTarget>=1300)
            {
                extPercentage = 1-(extTarget-1400.0)/1500;
                spin.setPosition(0.1528);
                smallWrist.setPosition(0.1517 + ((0.2267-0.1517)*extPercentage));
                bigWristL.setPosition(0.71 + ((0.74-0.71)*extPercentage));
                bigWristR.setPosition(0.2894 - ((0.2894-0.2594)*extPercentage));
                //smallWrist - 0.1517 - 0.2267
                //bigWristL - 0.71 - 0.74
                //bigWristR - 0.2894 - 0.2594
            }*/
            //endregion

            //region FLIPPER
            if(gamepad2.dpad_left && flpPosTarget>=0)
            {
                telemetry.addLine("flp UP");
                currentState = poseControlState.FREE;

                if(flpPosTarget-20<=0)
                {
                    flpPosTarget-=Math.abs(flpPosTarget);
                }
                else {
                    flpPosTarget-=20;
                }
            }
            else if(gamepad2.dpad_right && flpPosTarget<1620 && !(extTarget>500 && flpPosTarget<1200))
            {
                telemetry.addLine("flp DOWN");
                currentState = poseControlState.FREE;

                if(flpPosTarget+20>=1620)
                {
                    flpPosTarget+=Math.abs(1620-flpPosTarget);
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
        }
        else {
            setStates();
            //region CLAW
            if (currG2.b && !oldG2.b)
            {
                if(clawIH)
                {
                    claw.setPosition(0.6);
                }
                else {
                    claw.setPosition(0.3);
                }
                clawIH = !clawIH;
            }
            //endregion
        }
    }
    public void setStates()
    {
        //region HIGH POSITION
        if(currentState!= poseControlState.HIGH && currG2.y && !oldG2.y)
        {
            telemetry.addLine("HIGH POSITION");
            currentState = poseControlState.HIGH;
            if(flpPosTarget<200) {
                extTarget = 1160;
                flpPosTarget = 0;
                spin.setPosition(0.1567);
                bigWristR.setPosition(0.88);
                bigWristL.setPosition(0.1194);
                smallWrist.setPosition(0.1567);
            }
            else {
                extTarget = 0;
                while(-extLMotor.getCurrentPosition()>10) {
                    driverAControls();
                    extCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                flpPosTarget = 0;
                jerkTimer.reset();
                while(jerkTimer.time() < 1) {
                    driverAControls();
                    extCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                spin.setPosition(0.1567);
                bigWristR.setPosition(0.88);
                bigWristL.setPosition(0.1194);
                smallWrist.setPosition(0.1567);
                extTarget = 1160;
                jerkTimer.reset();
                while(jerkTimer.time() < 0.5) {
                    driverAControls();
                    extCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
            }
        }
        //endregion

        //region LOW POSITION
        if(currentState!= poseControlState.LOW && currG2.dpad_down && !oldG2.dpad_down)
        {
            telemetry.addLine("LOW POSITION");
            currentState = poseControlState.LOW;
            if(flpPosTarget<200) {
                extTarget = 420;
                flpPosTarget = 0;
                spin.setPosition(0.1567);
                bigWristR.setPosition(0.88);
                bigWristL.setPosition(0.1194);
                smallWrist.setPosition(0.1567);
            }
            else {
                extTarget = 0;
                jerkTimer.reset();
                while(jerkTimer.time() < 0.3) {
                    driverAControls();
                    extCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                flpPosTarget = 0;
                jerkTimer.reset();
                while(jerkTimer.time() < 1) {
                    driverAControls();
                    extCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                spin.setPosition(0.1567);
                bigWristR.setPosition(0.88);
                bigWristL.setPosition(0.1194);
                smallWrist.setPosition(0.1567);
                extTarget = 420;
                jerkTimer.reset();
                while(jerkTimer.time() < 0.5) {
                    driverAControls();
                    extCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
            }
        }
        //endregion

        //region HOME POSITION
        if(currentState!= poseControlState.HOME && (currG2.dpad_right && !oldG2.dpad_right))
        {
            telemetry.addLine("HOME POSITION");
            currentState = poseControlState.HOME;
            if(flpPosTarget<200) {
                extTarget = 0;
                flpPosTarget = 0;
                spin.setPosition(0.1567);
                bigWristR.setPosition(0.88);
                bigWristL.setPosition(0.1194);
                smallWrist.setPosition(0.1567);
            }
            else {
                extTarget = 0;
                jerkTimer.reset();
                while(jerkTimer.time() < 0.3) {
                    driverAControls();
                    extCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                flpPosTarget = 0;
                jerkTimer.reset();
                while(jerkTimer.time() < 1) {
                    driverAControls();
                    extCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                spin.setPosition(0.1567);
                bigWristR.setPosition(0.88);
                bigWristL.setPosition(0.1194);
                smallWrist.setPosition(0.1567);
            }
        }
        //endregion

        //region WALL POSITION
        if(currentState!= poseControlState.WALL && currG2.x && !oldG2.x) {
            telemetry.addLine("WALL POSITION");
            currentState = poseControlState.WALL;
            if(flpPosTarget>1200) {
                extTarget = 0;
                flpPosTarget = 1650;
                spin.setPosition(0.1528);
                bigWristR.setPosition(0.48);
                bigWristL.setPosition(0.5183);
                smallWrist.setPosition(0.3206);
            }
            else {
                extTarget = 0;
                jerkTimer.reset();
                while(jerkTimer.time() < 0.3) {
                    driverAControls();
                    extCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                flpPosTarget = 1650;
                jerkTimer.reset();
                while(jerkTimer.time() < 1) {
                    driverAControls();
                    extCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                spin.setPosition(0.1528);
                bigWristR.setPosition(0.48);
                bigWristL.setPosition(0.5183);
                smallWrist.setPosition(0.3206);
            }
        }
        //endregion

        //region PICKUP POSITION
        if(currentState!= poseControlState.PICKUP && currG2.a && !oldG2.a)
        {
            telemetry.addLine("PICKUP POSITION");
            currentState = poseControlState.PICKUP;
            if(flpPosTarget>1200) {
                extTarget = 560;
                flpPosTarget = 1650;
                spin.setPosition(0.1528);
                bigWristR.setPosition(0.28);
                bigWristL.setPosition(0.7194);
                smallWrist.setPosition(0.2061);
            }
            else {
                extTarget = 0;
                jerkTimer.reset();
                while(jerkTimer.time() < 0.3) {
                    driverAControls();
                    extCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                flpPosTarget = 1650;
                jerkTimer.reset();
                while(jerkTimer.time() < 1) {
                    driverAControls();
                    extCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                spin.setPosition(0.1528);
                bigWristR.setPosition(0.28);
                bigWristL.setPosition(0.7194);
                smallWrist.setPosition(0.2061);
                extTarget = 560;
                jerkTimer.reset();
                while(jerkTimer.time() < 0.5) {
                    driverAControls();
                    extCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
            }
        }
        //endregion

        //region JERK
        if( flipMotor.getCurrentPosition()>-2000 && currG2.left_bumper && !oldG2.left_bumper)
        {
            telemetry.addLine("JERK");
            if(extTarget>=300) {
                extTarget = extTarget - 300;
                jerkTimer.reset();
                while (jerkTimer.time() < 0.3) {
                    driverAControls();
                    extCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
            }
            claw.setPosition(0.3);
        }
        //endregion

        //region HIGHBASKET POSITION
        if(currentState!= poseControlState.HIGHBASKET && currG2.dpad_up && !oldG2.dpad_up)
        {
            telemetry.addLine("HIGHBASKET POSITION");
            currentState = poseControlState.HIGHBASKET;
            if(flpPosTarget<200) {
                extTarget = 1160;
                flpPosTarget = 0;
                spin.setPosition(0.1567);
                bigWristR.setPosition(0.88);
                bigWristL.setPosition(0.1194);
                smallWrist.setPosition(0.1567);
            }
            else {
                extTarget = 0;
                jerkTimer.reset();
                while(jerkTimer.time() < 0.3) {
                    driverAControls();
                    extCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                flpPosTarget = 0;
                jerkTimer.reset();
                while(jerkTimer.time() < 1) {
                    driverAControls();
                    extCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                spin.setPosition(0.1567);
                bigWristR.setPosition(0.88);
                bigWristL.setPosition(0.1194);
                smallWrist.setPosition(0.1567);
                extTarget = 1160;
                jerkTimer.reset();
                while(jerkTimer.time() < 0.5) {
                    driverAControls();
                    extCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
            }
        }
        //endregion

        //region LOWBASKET POSITION
        if(currentState!= poseControlState.LOWBASKET && currG2.dpad_left && !oldG2.dpad_left)
        {
            telemetry.addLine("LOWBASKET POSITION");
            currentState = poseControlState.LOWBASKET;
            if(flpPosTarget<200) {
                extTarget = 420;
                flpPosTarget = 0;
                spin.setPosition(0.1567);
                bigWristR.setPosition(0.88);
                bigWristL.setPosition(0.1194);
                smallWrist.setPosition(0.1567);
            }
            else {
                extTarget = 0;
                jerkTimer.reset();
                while(jerkTimer.time() < 0.3) {
                    driverAControls();
                    extCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                flpPosTarget = 0;
                jerkTimer.reset();
                while(jerkTimer.time() < 1) {
                    driverAControls();
                    extCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
                spin.setPosition(0.1567);
                bigWristR.setPosition(0.88);
                bigWristL.setPosition(0.1194);
                smallWrist.setPosition(0.1567);
                extTarget = 420;
                jerkTimer.reset();
                while(jerkTimer.time() < 0.5) {
                    driverAControls();
                    extCONTROLLER();
                    flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
                }
            }
        }
        //endregion
    }

    //CONTROLLERS
    public void extCONTROLLER()
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
        flipMotor.setVelocity(velocityVal);
    }

    //RANDOM
    public void RAINBOW(double speed)
    {
        for(double i=0; i<1; i+=0.01)
        {
            jerkTimer.reset();
            while(jerkTimer.time() < speed) {
                extCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            gamepad1.setLedColor(1, i, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(1, i, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
        for(double i=1; i>0; i-=0.01)
        {
            jerkTimer.reset();
            while(jerkTimer.time() < speed) {
                extCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            gamepad1.setLedColor(1, i, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(i, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        }
        for(double i=0; i<1; i+=0.01)
        {
            jerkTimer.reset();
            while(jerkTimer.time() < speed) {
                extCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            gamepad1.setLedColor(1, i, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(0, 1, i, Gamepad.LED_DURATION_CONTINUOUS);
        }
        for(double i=1; i>0; i-=0.01)
        {
            jerkTimer.reset();
            while(jerkTimer.time() < speed) {
                extCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            gamepad1.setLedColor(1, i, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(0, i, 1, Gamepad.LED_DURATION_CONTINUOUS);
        }
        for(double i=0; i<1; i+=0.01)
        {
            jerkTimer.reset();
            while(jerkTimer.time() < speed) {
                extCONTROLLER();
                flpCONTROLLER(flpPosTarget, flipMotor.getCurrentPosition());
            }
            gamepad1.setLedColor(1, i, 0, Gamepad.LED_DURATION_CONTINUOUS);
            gamepad2.setLedColor(i, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        }
        for(double i=1; i>0; i-=0.01)
        {
            jerkTimer.reset();
            while(jerkTimer.time() < speed) {
                extCONTROLLER();
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
        telemetry.addData("\next percent - ", extPercentage);


        telemetry.addData("\nspinner - ", spin.getPosition());
        telemetry.addData("small wrist - ", smallWrist.getPosition());
        telemetry.addData("big wrist right - ", bigWristR.getPosition());
        telemetry.addData("big wrist left - ", bigWristL.getPosition());
        telemetry.addData("claw - ", claw.getPosition());
    }
}