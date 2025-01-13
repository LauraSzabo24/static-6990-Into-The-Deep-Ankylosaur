package RelicsFromCenterstageGame.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import Autonomous.Mailbox;

//copied last 11/7/2023 12:47 am

@TeleOp
@Disabled
public class BlueTetrisLessTele extends OpMode {
    //drivetrain
    IMU imu;

    //PID material
    DcMotorEx slideMotorRight;
    DcMotorEx slideMotorLeft;
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double integralSum = 0;
    public static double Kp = 0.01;
    public static double Kd = 0.0;
    public static double targetPosition = 5000;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    public TelemetryPacket packet;


    //DRIVER A material
    //toggles
    private boolean armInHome;
    private boolean clawInHome;
    private boolean pushPopInHome;
    private boolean intakeLiftInHome;
    //other
    private boolean confirmA;
    private boolean manualOn;
    private boolean emergencyMode;
    private Servo clawServo, armLeftServo,armRightServo, airplaneServo, intakeLiftServo;
    DcMotorEx intakeMotor;

    //mecanum drive stuff
    private DcMotorEx motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
    private double getMax(double[] input) {
        double max = Integer.MIN_VALUE;
        for (double value : input) {
            if (Math.abs(value) > max) {
                max = Math.abs(value);
            }
        }
        return max;
    }

    //button controls for DRIVER A
    private boolean a1Pressed;
    private boolean a1Released;
    private boolean b1Pressed;
    private boolean b1Released;
    private boolean x1Pressed;
    private boolean x1Released;
    private boolean y1Pressed;
    private boolean y1Released;
    private boolean leftBumperReleased;
    private boolean leftBumperPressed;
    private boolean rightBumperReleased;
    private boolean rightBumperPressed;



    //DRIVER B material
    private static String[][] outputArray;
    private int cursorX;
    private int cursorY;
    private static int cursorFlash;
    private int[] firstPos;
    private int[] secPos;
    private boolean confirmB;
    private String[] colors;
    private String previousOutput;

    //button controls for DRIVER B
    private boolean a2Pressed;
    private boolean a2Released;
    private boolean b2Pressed;
    private boolean b2Released;
    private boolean x2Pressed;
    private boolean x2Released;
    private boolean y2Pressed;
    private boolean y2Released;


    //cursor movement DRIVER B
    private boolean leftPressed;
    private boolean leftReleased;
    private boolean rightPressed;
    private boolean rightReleased;
    private boolean downPressed;
    private boolean downReleased;
    private boolean upPressed;
    private boolean upReleased;
    private boolean boxRow;

    //mail
    IMU.Parameters parameters;
    String dirTestIMU;

    public void driverAInitialize()
    {
        //modes
        manualOn = true;
        emergencyMode = true; //false for tetris
        confirmA = false;

        //emergency mode/ button controls
        a1Pressed = false;
        a1Released = false;
        b1Pressed = false;
        b1Released = false;
        x1Pressed = false;
        x1Released = false;
        y1Pressed = false;
        y1Released = false;
        rightBumperPressed = false;
        rightBumperReleased = false;
        leftBumperPressed = false;
        leftBumperReleased = false;

        //dashboard
        dashboard.setTelemetryTransmissionInterval(25);

        //slide motors
        slideMotorLeft = hardwareMap.get(DcMotorEx.class, "LeftSlide");
        slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideMotorRight = hardwareMap.get(DcMotorEx.class, "RightSlide");
        slideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        targetPosition = 0;

        //drive motors
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("BL");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("FR");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("BR");

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //imu
        // Retrieve the IMU from the hardware map
        Mailbox mail =  new Mailbox();
        imu = hardwareMap.get(IMU.class, "imu");
        parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        //intake outake servos
        armInHome = true;
        pushPopInHome = true;
        clawInHome = true;
        intakeLiftInHome = true;
        clawServo = hardwareMap.get(Servo.class, "claw");
        armRightServo = hardwareMap.get(Servo.class, "armRightServo");
        armLeftServo = hardwareMap.get(Servo.class, "armLeftServo");
        airplaneServo = hardwareMap.get(Servo.class, "airplaneServo");
        intakeLiftServo = hardwareMap.get(Servo.class, "intakeLiftServo");

        //intake motor
        intakeMotor = (DcMotorEx) hardwareMap.dcMotor.get("intakeMotor");

        //start positions
        armLeftServo.setPosition(0.99);
        armRightServo.setPosition(0.01);

    }
    public void driverBInitialize()
    {
        outputArray = new String[12][13]; //original: 12 13 // new: 12 14
        cursorX = 1;
        cursorY = 10;
        cursorFlash = 50;
        firstPos = new int[]{-1,-1};
        secPos = new int[]{-1,-1};
        confirmB = false;
        previousOutput = "";
        boxRow = true;
        colors = new String[] {"", ""};
        makeGrid();
        printAll();

        //color
        getColors();
        a2Pressed = false;
        a2Released = false;
        b2Pressed = false;
        b2Released = false;

        //other button controls
        x2Pressed = false;
        x2Released = false;
        y2Pressed = false;
        y2Released = false;

        //cursor
        leftPressed = false;
        leftReleased = false;
        rightPressed = false;
        rightReleased = false;
        upPressed = false;
        upReleased = false;
        downPressed = false;
        downReleased = false;
    }
    public void cameraInit()
    {
        //color camera stuff goes in here
    }
    @Override
    public void init()
    {
        driverAInitialize();
        driverBInitialize();
        cameraInit();
    }
    @Override
    public void start()
    {
        makeGrid();
        //everything goes in here that isn't looping, won't be much
    }
    @Override
    public void loop()
    {
        //test
        //telemetry.addData("imu direction" + dirTestIMU, Mailbox.autoEndHead);

        //ONLY FOR MEET ONE/TWO
        emergencyMode = true;

        //button update
        updateDriverAButtons();

        //EMERGENCY MODE CONTROLS
        if(leftBumperReleased && rightBumperReleased && (gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_down))
        {
            leftBumperReleased = false;
            leftBumperPressed = false;
            rightBumperReleased = false;
            rightBumperPressed = false;
            /*b1Released = false;
            b1Pressed = false;
            a1Released = false;
            a1Pressed = false;
            x1Released = false;
            x1Pressed = false;
            y1Released = false;
            y1Pressed = false;*/
            if(emergencyMode)            {
                //emergencyMode = false;
            }
            else {
                //emergencyMode = true;
            }
            a2Released = false;
            a2Pressed = false;
        }
        if(emergencyMode)
        {
            telemetry.addLine(String.format("EMERGENCYYYYYYYY MODEEEEEEEEEE"));
            updateDriverBButtons();
            emergencyModeControls();
        }

        //Normal Driver A Controls
        if(!emergencyMode && manualOn)
        {
            //everything else
            updateDriverAControls();
            //confirmation
            if(b1Released)
            {
                b1Released = false;
                b1Pressed = false;
                confirmA = true;
            }
        }

        //Tetris Driver B Updating
        if(!emergencyMode) {
            printAll();
            updateTetrisThing();
        }

        //Tetris color checker
        if(colors[0].equals("") && colors[1].equals("") && confirmB && confirmA)
        {
            //getColors();
        }

        //Tetris Pixel Placing Thing
        if(confirmA && confirmB && !emergencyMode)
        {
            int[] place1 = firstPos;
            int[] place2 = secPos;
            runPixelPlacing(place1, place2);
            //firstPos = new int[]{-1,-1};
            //secPos = new int[]{-1,-1};
            //confirmB = false;
            //confirmA = false;
        }

        //telemetry CAN DELETE LATERRRRR
        if(armInHome){
            telemetry.addLine(String.format("arm in"));
        }
        else{
            telemetry.addLine(String.format("arm out"));
        }
        if(clawInHome){
            telemetry.addLine(String.format("claws in"));
        }
        else{
            telemetry.addLine(String.format("claws out"));
        }
        if(pushPopInHome){
            telemetry.addLine(String.format("push pop in"));
        }
        else{
            telemetry.addLine(String.format("push pop out"));
        }

        telemetry.addData("servo position in ", armLeftServo.getPosition());
        telemetry.update();
    }

    //DRIVER A NORMAL CONTROLS FROM HEREEEE
    public void updateDriverAControls()
    {
        //mecanum
        drive();

        //pivots
        if(gamepad1.right_bumper)
        {
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
            motorFrontLeft.setPower(1.5);//0.9
            motorFrontRight.setPower(-1.5);
            telemetry.addLine(String.format("pivot right"));

        }
        else if(gamepad1.left_bumper)
        {
            motorBackLeft.setPower(0);
            motorBackRight.setPower(0);
            motorFrontLeft.setPower(-1.5);
            motorFrontRight.setPower(1.5);
            telemetry.addLine(String.format("pivot left"));
        }

        //intake lift here
        /*if(y1Released && intakeLiftInHome)
        {
            y1Pressed = false;
            y1Released = false;
            intakeLiftInHome = false;
            intakeLiftServo.setPosition(0.75);
        }
        if(y1Released && !intakeLiftInHome) {
            y1Pressed = false;
            y1Released = false;
            intakeLiftInHome = true;
            intakeLiftServo.setPosition(0.25);
        }*/

        //intake spinner sucking vacuum cleaner thing
        if(gamepad1.a)
        {
            intakeMotor.setPower(0.6);
            telemetry.addLine(String.format("powering vacuum"));
        }
        else if(gamepad1.x)
        {
            intakeMotor.setPower(-0.6);
            telemetry.addLine(String.format("powering vacuum BACK"));
        }
        else{
            intakeMotor.setPower(0);
            telemetry.addLine(String.format("vacuum on standby"));
        }

        //telemetry CAN DELETE LATERRRR
        if(intakeLiftInHome){
            telemetry.addLine(String.format("intake lifted"));
        }
        else{
            telemetry.addLine(String.format("intake lowered"));
        }
    }
    public void drive()
    {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.options) {
            imu.resetYaw();
        }

        //strange math that somehow works
       // double botHeading = Math.toRadians(Mailbox.autoEndHead) - ((2*Math.PI) - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.toRadians(270));

       /* telemetry.addData("imu value", Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
        telemetry.addData("bot value", Math.toDegrees(botHeading));
        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);*/

        /*rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        motorFrontLeft.setPower(frontLeftPower/2);
        motorBackLeft.setPower(backLeftPower/2);
        motorFrontRight.setPower(frontRightPower/2);
        motorBackRight.setPower(backRightPower/2);
        telemetry.addLine(String.format("setting motor powers to:"));
        telemetry.addData("frontLeft ", frontLeftPower);
        telemetry.addData("backLeft ", backLeftPower);
        telemetry.addData("frontRight ", frontRightPower);
        telemetry.addData("backRight ", backRightPower);*/
    }







    //EMERGENCY MODE THINGS HEREEE
    public void emergencyModeControls()
    {
        updateDriverAControls();

        //ALL DRIVER B CONTROLS
        if(gamepad2.dpad_up && slideMotorLeft.getCurrentPosition()<3500 )
        {
            slideMotorLeft.setPower(0.7);
            slideMotorRight.setPower(0.7);
            telemetry.addLine(String.format("slide goes up"));
        }
        if(gamepad2.dpad_down && slideMotorLeft.getCurrentPosition() > 100)
        {
            slideMotorLeft.setPower(-0.4);
            slideMotorRight.setPower(-0.4);
            telemetry.addLine(String.format("slide goes down"));
        }
        if(!gamepad2.dpad_up && !gamepad2.dpad_down) //&& (Math.abs(targetPosition - slidePos)<15))
        {
            slideMotorLeft.setPower(0);
            slideMotorRight.setPower(0);
            telemetry.addLine(String.format("slide on standby"));
        }
        if(gamepad2.dpad_left)
        {
            telemetry.addLine(String.format("slide goes full down"));
        }
        //double power = returnPower(targetPosition, slideMotorLeft.getCurrentPosition());
        telemetry.addData("right motor position: ", slideMotorRight.getCurrentPosition());
        telemetry.addData("left motor position: ", slideMotorLeft.getCurrentPosition());
        //telemetry.addData("targetPosition: ", targetPosition);
        //telemetry.addData("power: ", power);


        /*slideMotorLeft.setPower(power);
        slideMotorRight.setPower(power);*/

        //flipping thing
        if(y2Released && armInHome)
        {
            y2Released = false;
            y2Pressed = false;
            armInHome = false;
            armLeftServo.setPosition(1);
            armRightServo.setPosition(0);
        }
        else if(y2Released) {
            y2Released = false;
            y2Pressed = false;
            armInHome = true;
            armLeftServo.setPosition(0);
            armRightServo.setPosition(1);
        }

        //push pop (not for meet 1)
        /*if(x2Released && pushPopInHome)
        {
            x2Released = false;
            x2Pressed = false;
            pushPopInHome = false;
            //pushPopServo.setPosition(0.5);
        } else if (x2Released)
        {
            x2Released = false;
            x2Pressed = false;
            pushPopInHome = true;
            //pushPopServo.setPosition(0);
        }*/

        //claw servo
        if(a2Released && clawInHome)
        {
            a2Released = false;
            a2Pressed = false;
            clawInHome = false;
            clawServo.setPosition(0);
        } else if (a2Released)
        {
            a2Released = false;
            a2Pressed = false;
            clawInHome = true;
            clawServo.setPosition(0.3);
        }

        //airplane
        if(gamepad2.left_bumper && gamepad2.right_bumper)
        {
            airplaneServo.setPosition(0.2);
        }

        //telemetry CAN DELETE LATERRRRR
        telemetry.addData("servo is at", clawServo.getPosition());
        telemetry.update();
        if(armInHome){
            telemetry.addLine(String.format("arm in"));
        }
        else{
            telemetry.addLine(String.format("arm out"));
        }
        if(clawInHome){
            telemetry.addLine(String.format("claws in"));
        }
        else{
            telemetry.addLine(String.format("claws out"));
        }
        if(pushPopInHome){
            telemetry.addLine(String.format("push pop in"));
        }
        else{
            telemetry.addLine(String.format("push pop out"));
        }
    }

    //BUTTON UPDATES
    public void updateDriverAButtons()
    {
        //a
        if(gamepad1.a)
        {
            a1Pressed = true;
        }
        else if(a1Pressed){
            a1Released = true;
        }
        //b
        if(gamepad1.b)
        {
            b1Pressed = true;
        }
        else if(b1Pressed){
            b1Released = true;
        }
        //x
        if(gamepad1.x)
        {
            x1Pressed = true;
        }
        else if(x1Pressed){
            x1Released = true;
        }
        //y
        if(gamepad1.y)
        {
            y1Pressed = true;
        }
        else if(y1Pressed){
            y1Released = true;
        }
        //right bumper
        if(gamepad1.right_trigger>0.8)
        {
            rightBumperPressed = true;
        }
        else if(rightBumperPressed){
            rightBumperReleased = true;
        }
        //left bumper
        if(gamepad1.left_trigger>0.8)
        {
            leftBumperPressed = true;
        }
        else if(leftBumperPressed){
            leftBumperReleased = true;
        }
    }
    public void updateDriverBButtons()
    {
        //x
        if(gamepad2.x)
        {
            x2Pressed = true;
        }
        else if(x2Pressed){
            x2Released = true;
        }
        //y
        if(gamepad2.y)
        {
            y2Pressed = true;
        }
        else if(y2Pressed){
            y2Released = true;
        }
        //a
        if(gamepad2.a)
        {
            a2Pressed = true;
        }
        else if(a2Pressed){
            a2Released = true;
        }
    }






    //ALL TETRIS TRASH DOWN FROM HERE
    public int xCoorSimplify(int xCoor, int yCoor) //doesn't work
    {
        //remove spaces from x coor
        int newX = 0;
        for(int x = 0; x<outputArray[0].length; x++)
        {
            if(!outputArray[yCoor][x].equals("   ") && !outputArray[yCoor][x].equals("_."))
            {
                newX+=1;
            }
        }
        return newX;
    }
    public double convertX(int xCoor, int yCoor) //doesn't work
    {
        telemetry.addLine(String.format("Old x coor" + xCoor));
        xCoor = xCoorSimplify(xCoor, yCoor);
        telemetry.addLine(String.format("New x coor" + xCoor));
        if(boxRow)
        {
            return 0.3125+(3*xCoor*0.5);
            //return distanceToFirstTopPixelFromScrew+(pixelWidth*xCoor*0.5);
        }
        else {
            return 1.8125+(3*xCoor*0.5);
            //return distanceToFirstPixel+(pixelWidth*xCoor*0.5);
        }
    }
    public double convertY(int yCoor) //seems to work
    {
        double inches = 10.5;
        for(int x=10; x>=yCoor; x--)
        {
            if(x%2==1)
            {
                inches+=2;
            }
            else {
                inches+=3;
            }
        }
        return inches;
    }
    public void runPixelPlacing(int [] target1, int [] target2)
    {
        manualOn = false;
        double[] position1 = new double[2];
        double[] position2 = new double[2];

        telemetry.addLine(String.format("" + target1[0]));
        if(target1[0]!=-1) {
            telemetry.addLine(String.format("x normal " + target1[1]));
            telemetry.addLine(String.format("y normal " + target1[0]));
            position1[0] = convertX(target1[1], target1[0]);
            position1[1] = convertY(target1[0]);

            telemetry.addLine(String.format("x in inches " + position1[0]));
            telemetry.addLine(String.format("y in inches " + position1[1]));
            //road runner code that goes to the correct firstPos x in arc shape
            //moves slides while going
            //push pixel out

            if(target2[0]!=-1) {
                telemetry.addLine(String.format("x normal " + target2[1]));
                telemetry.addLine(String.format("y normal " + target2[0]));
                position2[0] = convertX(target2[1], target2[0]);
                position2[1] = convertY(target2[0]);
                telemetry.addLine(String.format("x in inches " + position2[0]));
                telemetry.addLine(String.format("y in inches " + position2[1]));
            }
        }

        //put early end / overriding in here
        //manualOn = true;
    }

    public void getColors()
    {
        colors = new String[] {"G", "P"}; //⬜ 🟪 🟩 🟨
    }
    public void updateTetrisThing()
    {
        //cursor flashing
        if(outputArray[cursorY][cursorX]!="◼")
        {
            previousOutput = outputArray[cursorY][cursorX];
        }
        cursorFlash--;
        if(cursorFlash>25) {
            outputArray[cursorY][cursorX] = "◼"; //⬛ █◼
        }
        else
        {
            outputArray[cursorY][cursorX] = previousOutput;
        }
        if(cursorFlash<1)
        {
            cursorFlash=50;
            previousOutput = outputArray[cursorY][cursorX];
            outputArray[cursorY][cursorX] = previousOutput;
        }
        cursorUpdate();


        //selection
        if(a2Released && manualOn && !(colors[0].equals("")))
        {
            a2Pressed = false;
            a2Released = false;
            outputArray[cursorY][cursorX] = colors[0];
            if(colors[1]=="")
            {
                secPos = new int[]{cursorY, cursorX};
            }
            else{
                firstPos = new int[]{cursorY, cursorX};
            }
            colors[0]=colors[1];
            colors[1]="";
        }

        //retrieval???? necessary???

        //confirmation
        if(b2Released && (firstPos[1]!=-1 || secPos[1]!=-1) && manualOn)
        {
            b2Pressed = false;
            b2Released = false;
            confirmB = true;
        }
    }
    public void cursorUpdate() //WORKS
    {
        //left
        if(gamepad2.dpad_left)
        {
            leftPressed = true;
        }
        else if(leftPressed){
            leftReleased = true;
        }
        //right
        if(gamepad2.dpad_right && cursorX<12)
        {
            rightPressed = true;
        }
        else if(rightPressed){
            rightReleased = true;
        }
        //down
        if(gamepad2.dpad_down && cursorY<10)
        {
            downPressed = true;
        }
        else if(downPressed){
            downReleased = true;
        }
        //up
        if(gamepad2.dpad_up && cursorY>=1)
        {
            upPressed = true;
        }
        else if(upPressed){
            upReleased = true;
        }

        //a and b
        if(gamepad2.a)
        {
            a2Pressed = true;
        }
        else if(a2Pressed){
            a2Released = true;
        }
        //up
        if(gamepad2.b)
        {
            b2Pressed = true;
        }
        else if(b2Pressed){
            b2Released = true;
        }

        //cursor movement
        isBoxRow();
        if(leftReleased && cursorX>1)
        {
            leftPressed = false;
            leftReleased = false;
            outputArray[cursorY][cursorX] = previousOutput;
            if(cursorX-1>1){
                cursorX-=2;
            }
        }
        else if(rightReleased && cursorX<12)
        {
            rightPressed = false;
            rightReleased = false;
            outputArray[cursorY][cursorX] = previousOutput;
            if(cursorX+1<12){
                cursorX+=2;
            }
        }
        else if(downReleased && cursorY<10)
        {
            downPressed = false;
            downReleased = false;
            outputArray[cursorY][cursorX] = previousOutput;
            cursorY++;
            if(boxRow){
                cursorX++;
            }
            else{
                cursorX--;
            }
        }
        else if(upReleased && cursorY>=1)
        {
            upPressed = false;
            upReleased = false;
            outputArray[cursorY][cursorX] = previousOutput;
            cursorY--;
            if(boxRow){
                cursorX++;
            }
            else{
                cursorX--;
            }
        }
    }
    public void isBoxRow() //WORKS
    {
        if(cursorY%2==0)
        {
            boxRow = true;
        }
        else{
            boxRow = false;
        }
    }
    public void printAll()
    {
        //colors avaliable
        telemetry.addLine(String.format("                    1      2"));
        telemetry.addLine(String.format("COLORS - "+colors[0]+"      "+colors[1]));

        //2d array output
        String rowOut = "";
        for(int r=0; r<outputArray.length; r++)
        {
            rowOut = "";
            for(int c=0; c<outputArray[1].length; c++) {
                rowOut += outputArray[r][c];
            }
            telemetry.addData("", rowOut);
        }

        //selections queue
        if(firstPos[0]!=-1 && secPos[0]!=-1)
        {
            telemetry.addLine(String.format("QUEUE |   "+firstPos[1]+","+firstPos[0]+" then "+secPos[1]+","+secPos[0], null));
        }
        else if(firstPos[0]!=-1)
        {
            telemetry.addLine(String.format("QUEUE |   "+firstPos[1]+","+firstPos[0]));
        }
        else {
            telemetry.addLine(String.format("QUEUE |   "));

        }

        //confirmation queue
        if(confirmB && confirmA)
        {
            telemetry.addLine(String.format("ROBOT RUNNING"));
        }
        else if(confirmB)
        {
            telemetry.addLine(String.format("CONFIRMED PLEASE WAIT"));
        }
        else if(colors[1].equals("") && firstPos[0]!=-1)
        {
            telemetry.addLine(String.format("UNCONFIRMED CHANGES"));
        }
        else if(colors[0].equals("") && colors[1].equals(""))
        {
            telemetry.addLine(String.format("NO PIXELS LOADED"));
        }
        else
        {
            telemetry.addLine(String.format("PIXELS READY TO GO"));
        }

    }
    public void makeGrid() { //WORKS
        for (int r = 0; r < outputArray.length; r++) {
            for (int c = 0; c < outputArray[1].length; c++) {
                if (c != 0 && c != 14) {
                    if (r % 2 == 0 && c % 2 == 0) {
                        outputArray[r][c] = "   ";
                    } else if (r % 2 == 1 && c % 2 == 1) {
                        outputArray[r][c] = "   ";
                    } else {
                        outputArray[r][c] = "◻"; //O
                    }
                } else {
                    outputArray[r][c] = "   ";
                }
                if ((r == 2 || r == 5 || r == 8) && outputArray[r][c] == "   ") {
                    outputArray[r][c] = "_.";
                }
                if (r == 11) {
                    if (c == 3 || c == 7 || c == 11) {
                        outputArray[r][c] = "X";
                    } else {
                        outputArray[r][c] = "_";
                    }
                }
            }
        }
    }
}