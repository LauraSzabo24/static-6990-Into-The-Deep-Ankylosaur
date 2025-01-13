package RelicsFromCenterstageGame.TeleOp.Tetris;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.NewMecanumDrive;

import Autonomous.Mailbox;

@TeleOp
@Disabled
public class BlueSemiTetris extends LinearOpMode {
    /* TO-DO
    - make airplane launcher toggle
    - make airplane launcher in manual
    - tetris x
    - tetris y
    - tetris z
    - tetris little push
     */

    //GLOBAL VARIABLES
    //region TEMPORARY
    Vector2d targetAVector = new Vector2d(45, 45);
    double targetAHeading = Math.toRadians(90);
    Vector2d targetBVector = new Vector2d(-15, 25);
    double targetAngle = Math.toRadians(45);
    //endregion
    //region GENERAL
    enum Mode {
        MANUAL,
        AUTO,
        EMERGENCY,
        HANGING
    }
    Mode state = Mode.MANUAL;
    Pose2d poseEstimate;
    Pose2d currentLocation;

    double poseTester;
    public boolean pathComplete;
    //endregion
    //region MOTORS AND SERVOS
    NewMecanumDrive drive;
    private Servo clawServo, armLeftServo, armRightServo, airplaneServo, flickerServo;
    DcMotorEx intakeMotor, slideMotorLeft, slideMotorRight;

    double xShiftMinimizer;
    //endregion
    //region DRIVER A BUTTON CONTROLS
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
    //endregion
    //region DRIVER B BUTTON CONTROLS
    private boolean a2Pressed;
    private boolean a2Released;
    private boolean b2Pressed;
    private boolean b2Released;
    private boolean x2Pressed;
    private boolean x2Released;
    private boolean y2Pressed;
    private boolean y2Released;
    //endregion
    //region DRIVER B VARIABLES
    private static String[][] outputArray;
    private int cursorX;
    private int cursorY;
    private static int cursorFlash;
    private int[] firstPos;
    private int[] secPos;
    private boolean confirmB;
    private String[] colors;
    private String previousOutput;
    private double[] position1;
    private double[] position2;
    //endregion
    //region CURSER MOVEMENT
    private boolean leftPressed;
    private boolean leftReleased;
    private boolean rightPressed;
    private boolean rightReleased;
    private boolean downPressed;
    private boolean downReleased;
    private boolean upPressed;
    private boolean upReleased;
    private boolean boxRow;
    //endregion
    //region DRIVER A VARIABLES
    private double speed;
    private double multiply;
    private boolean armInHome;
    private boolean clawInHome;
    private boolean flickerHome;
    private boolean pushPopInHome;
    private boolean intakeLiftInHome;
    private boolean confirmA;
    //endregion


    //region MAIN
    @Override
    public void runOpMode() throws InterruptedException
    {
        //INITIALIZE
        drive = new NewMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(Mailbox.currentPose);
        drive.setPoseEstimate(new Pose2d(Mailbox.currentPose.getX(), Mailbox.currentPose.getY(), Mailbox.currentPose.getHeading() + Math.toRadians(-90))); // + Math.toRadians(180)
        //drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading() + Math.toRadians(-180))); // + Math.toRadians(180)
        driverAInitialize();
        driverBInitialize();
        pathComplete = false;


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            mainLoop();
        }
    }

    public void mainLoop()
    {
        //ANTISHIFT
        telemetry.addData("motor velocity average ", drive.secretAdditionMotorAverage());
        xShiftMinimizer -= (drive.secretAdditionMotorAverage()/45000); //25
        telemetry.addData("pose x tester ", poseTester);

        //UPDATES
        updateDriverAButtons();
        updateDriverBButtons();
        drive.update();
        telemetry.update();
        poseEstimate = new Pose2d(drive.getPoseEstimate().getX() + xShiftMinimizer, drive.getPoseEstimate().getY() + xShiftMinimizer, drive.getPoseEstimate().getHeading());

        //TELEMETRY
        telemetry.addData("mode", state);
        telemetry.addData("current x", poseEstimate.getX());
        telemetry.addData("current y", poseEstimate.getY());
        telemetry.addData("current heading", poseEstimate.getHeading());
        telemetry.addData("isDriveBusy ", drive.isBusy());

        telemetry.addData("start x ", currentLocation.getX());
        telemetry.addData("start y ", currentLocation.getY());
        telemetry.addData("start head ", currentLocation.getHeading());

        switch (state) {
            case MANUAL:
                emergencyModeControls();
                //DRIVER A
                if (b1Released) {
                    b1Released = false;
                    b1Pressed = false;
                    confirmA = true;
                }

                //TRANSFER TO HANGING | Driver B stick buttons
                if (gamepad2.right_stick_button || gamepad2.left_stick_button) {
                    state = Mode.HANGING;
                }

                //DRIVER B | updates and prints
                //printAll();
                //updateTetrisThing();
                //confirmation
                if (b2Released) {
                    b2Pressed = false;
                    b2Released = false;
                    confirmB = true;
                    firstPos = new int[] {0,0};
                }
                if (x2Released) {
                    x2Pressed = false;
                    x2Released = false;
                    confirmB = true;
                    firstPos = new int[] {8,0};
                }

                //Tetris Pixel Placing | checks for confirmation of both drivers | runs first position only
                /*if (confirmA && confirmB) {
                    int[] place1 = firstPos;
                    int[] place2 = secPos;
                    runPixelPlacing1(place1);
                    state = Mode.AUTO;
                }*/
                break;
            case AUTO:
                if (gamepad1.x) {
                    //drive.breakFollowing();
                    state = Mode.MANUAL;
                    resetTetris();
                }
                if (!drive.isBusy()) {
                    state = Mode.MANUAL;
                    resetTetris();
                }
                break;
            case EMERGENCY:
                telemetry.addLine(String.format("EMERGENCY MODE/n START DRIVING"));
                emergencyModeControls();

                //TRANSFER TO HANGING | Driver B stick buttons
                if (gamepad2.right_stick_button || gamepad2.left_stick_button) {
                    state = Mode.HANGING;
                }
                break;
            case HANGING:
                telemetry.addLine(String.format("HANGING"));
                slideMotorLeft.setPower(-0.4);
                slideMotorRight.setPower(-0.4);
                break;
        }
    }
    //endregion

    //region DRIVER A CONTROLS
    public void driving() //Field Centric Driving
    {
        //drivetrain changes in speed | right trigger slow | left trigger fast
        multiply = 1;
        speed = 3;
        if (gamepad1.right_trigger > 0) {
            multiply = 0.5;
            speed = 3;
        }
        if (gamepad1.left_trigger > 0) {
            multiply =0.7;
            speed = 1;
        }

        poseEstimate = drive.getPoseEstimate();

        Vector2d input = new Vector2d(
                -((gamepad1.left_stick_y)* multiply)/speed,
                -((gamepad1.left_stick_x)* multiply)/speed
        ).rotated(-poseEstimate.getHeading() + Math.toRadians(0));

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        ((gamepad1.right_stick_x * multiply)/speed)
                )
        );

        drive.update();
    }
    public void updateDriverAControls()
    {
        driving();

        //intake spinner | A button in | X button out
        if (gamepad1.a) {
            intakeMotor.setPower(0.6);
            telemetry.addLine(String.format("powering vacuum cleaner"));
        } else if (gamepad1.x) {
            intakeMotor.setPower(-0.6);
            telemetry.addLine(String.format("powering vacuum cleaner BACK"));
        } else {
            intakeMotor.setPower(0);
            telemetry.addLine(String.format("vacuum cleaner on standby"));
        }

        //More Telemetry
        if (intakeLiftInHome) {
            telemetry.addLine(String.format("intake lifted"));
        } else {
            telemetry.addLine(String.format("intake lowered"));
        }
        if (flickerHome) {
            telemetry.addLine(String.format("pixel device lifted"));
        } else {
            telemetry.addLine(String.format("pixel device lowered"));
        }
    }
    //endregion

    //region EMERGENCY
    public void emergencyModeControls() {
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

        //flipping thing | Y button
        if (y2Released && armInHome) {
            y2Released = false;
            y2Pressed = false;
            armInHome = false;
            flickerServo.setPosition(0.65);
            armLeftServo.setPosition(0.95);
            armRightServo.setPosition(0.05);
        } else if (y2Released) {
            y2Released = false;
            y2Pressed = false;
            armInHome = true;
            armLeftServo.setPosition(0);
            armRightServo.setPosition(1);
        }


        //flicker | X button
        if(armInHome) {
            if (x2Released && flickerHome) {
                x2Released = false;
                x2Pressed = false;
                flickerHome = false;

                flickerServo.setPosition(0.65);
            } else if (x2Released) {
                x2Released = false;
                x2Pressed = false;
                flickerHome = true;

                flickerServo.setPosition(0.22);
            }
        }

        //Arm low position | X button
        /*if (x2Released && pushPopInHome) {
            x2Released = false;
            x2Pressed = false;
            pushPopInHome = false;

            armLeftServo.setPosition(0.7);
            armRightServo.setPosition(0.3);
        } else if (x2Released) {
            x2Released = false;
            x2Pressed = false;
            pushPopInHome = true;

            armLeftServo.setPosition(0.7);
            armRightServo.setPosition(0.3);
        }*/

        //claw servo | A button
        if (a2Released && clawInHome) {
            a2Released = false;
            a2Pressed = false;
            clawInHome = false;
            clawServo.setPosition(0);
        } else if (a2Released) {
            a2Released = false;
            a2Pressed = false;
            clawInHome = true;
            clawServo.setPosition(0.3);
        }

        //airplane | Bumpers
        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            airplaneServo.setPosition(0.5);
        }

        //Telemetry
        telemetry.addData("servo is at", clawServo.getPosition());
        telemetry.update();
        if (armInHome) {
            telemetry.addLine(String.format("arm in"));
        } else {
            telemetry.addLine(String.format("arm out"));
        }
        if (clawInHome) {
            telemetry.addLine(String.format("claws in"));
        } else {
            telemetry.addLine(String.format("claws out"));
        }
        if (pushPopInHome) {
            telemetry.addLine(String.format("push pop in"));
        } else {
            telemetry.addLine(String.format("push pop out"));
        }
    }
    //endregion

    //region INITIALIZE
    public void driverAInitialize() {
        //modes
        confirmA = false;

        //drivetrain
        speed = 2;
        multiply = 1;

        //intake motor
        intakeMotor = (DcMotorEx) hardwareMap.dcMotor.get("intakeMotor");
        flickerServo = hardwareMap.get(Servo.class, "flickerServo");

        flickerServo.setPosition(0.65);

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
        flickerHome = false;
    }
    public void driverBInitialize() {
        //tetris material
        poseTester = 0;
        currentLocation = new Pose2d(0,0,0);
        position1 = new double[]{-1,-1};
        position2 = new double[]{-1,-1};
        outputArray = new String[12][14]; //original: 12 13 // new: 12 14
        cursorX = 1;
        cursorY = 10;
        cursorFlash = 50;
        firstPos = new int[]{-1, -1};
        secPos = new int[]{-1, -1};
        confirmB = false;
        previousOutput = "";
        boxRow = true;
        colors = new String[]{"@", "@"};
        makeGrid();
        printAll();

        //color
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

        //intake outake servos
        armInHome = true;
        pushPopInHome = true;
        clawInHome = true;
        intakeLiftInHome = true;
        clawServo = hardwareMap.get(Servo.class, "claw");
        armRightServo = hardwareMap.get(Servo.class, "armRightServo");
        armLeftServo = hardwareMap.get(Servo.class, "armLeftServo");
        airplaneServo = hardwareMap.get(Servo.class, "airplaneServo");

        //arm into start position
        armLeftServo.setPosition(0.99);
        armRightServo.setPosition(0.01);
    }
    //endregion

    //region BUTTON UPDATES
    public void updateDriverAButtons() {
        //a
        if (gamepad1.a) {
            a1Pressed = true;
        } else if (a1Pressed) {
            a1Released = true;
        }
        //b
        if (gamepad1.b) {
            b1Pressed = true;
        } else if (b1Pressed) {
            b1Released = true;
        }
        //x
        if (gamepad1.x) {
            x1Pressed = true;
        } else if (x1Pressed) {
            x1Released = true;
        }
        //y
        if (gamepad1.y) {
            y1Pressed = true;
        } else if (y1Pressed) {
            y1Released = true;
        }
        //right bumper
        if (gamepad1.right_trigger > 0.8) {
            rightBumperPressed = true;
        } else if (rightBumperPressed) {
            rightBumperReleased = true;
        }
        //left bumper
        if (gamepad1.left_trigger > 0.8) {
            leftBumperPressed = true;
        } else if (leftBumperPressed) {
            leftBumperReleased = true;
        }
    }
    public void updateDriverBButtons() {
        //x
        if (gamepad2.x) {
            x2Pressed = true;
        } else if (x2Pressed) {
            x2Released = true;
        }
        //y
        if (gamepad2.y) {
            y2Pressed = true;
        } else if (y2Pressed) {
            y2Released = true;
        }
        //a
        if (gamepad2.a) {
            a2Pressed = true;
        } else if (a2Pressed) {
            a2Released = true;
        }
    }
    //endregion

    //region TETRIS PATH GENERATION
    public void resetTetris()
    {
        if (colors[0].equals("") || colors[1].equals("")) {
            colors = new String[]{"@", "@"};
        }
        confirmB = false;
        confirmA = false;
        firstPos = new int[]{-1,-1};
        secPos = new int[]{-1,-1};
    }

    public void runPixelPlacing1(int[] target1) {
        position1 = new double[2];

        if (target1[0] != -1) {
            position1[0] = convertX(target1[1], target1[0]);
            position1[1] = convertY(target1[0]);

            //Move To Coordinate
            moveToPose(position1, 0);
            //Move Slides
            //Place Pixel
        }
    }
    public void moveToPose(double[] position, double zOffset)
    {
        pathComplete = false;
        poseTester = position[0];
        Vector2d positionVector = new Vector2d(-position[0], 45); //40.3 -45
        Pose2d start = poseEstimate;
        currentLocation = start;
        drive.setPoseEstimate(start);

        //while((poseEstimate.getY()-(-74))<5) {
        if (!drive.isBusy()) {
            Trajectory path = drive.trajectoryBuilder(start)
                    .lineToLinearHeading(new Pose2d(positionVector.getX(),positionVector.getY(),Math.toRadians(90)))
                    .build();
            drive.followTrajectory(path);
        }
        //}
        pathComplete = true;
    }

    public double convertX(int xCoor, int yCoor)
    {
        double inches = 0; //distance to last x coordinate unindented 40.3
        double toBoard = 21.3;
        if(xCoor%2==1) //indented
        {
            inches += 1.5; //distance between indented and unindented
            for(int i=13; i>xCoor; i--)
            {
                if(i%2==1)
                {
                    inches += 3.05; //one pixel
                }
            }
        }
        else{
            inches +=3.05; //idk it fixed it
            for(int i=13; i>xCoor; i--)
            {
                if(i%2==0)
                {
                    inches += 3.05; //one pixel
                }
            }
        }
        //inches = 40.3-inches;
        double meepMeepUnit = 1; //inches in meep meepian
        return inches + toBoard;
    }
    public double convertY(int yCoor)
    {
        double inches = 10.5; //height for first unindented pixel
        if(yCoor%2==1) //indented
        {
            inches += 2.03; //height between indented and unindented
        }
        for(int i=10; i>yCoor; i--)
        {
            if((yCoor%2==1) && (i%2==1)) //indented
            {
                inches += 4.03; //height between two pixels indented
            }
            else if((yCoor%2==0) && (i%2==0)) //indented
            {
                inches += 4.25; //height between two pixels unindented
            }
        }
        double slideValue = 1.67; // inches to slide value
        double initialSlideValue = 0; //0 position for slides
        return (inches * slideValue) + initialSlideValue;
    }
    public void convertZ() //add later
    {

    }
    //endregion

    //region TETRIS TELEMETRY
    public void updateTetrisThing()
    {
        //cursor flashing
        if (outputArray[cursorY][cursorX] != "◼") {
            previousOutput = outputArray[cursorY][cursorX];
        }
        cursorFlash--;
        if (cursorFlash > 3) {
            outputArray[cursorY][cursorX] = "◼"; //⬛ █◼
        } else {
            outputArray[cursorY][cursorX] = previousOutput;
        }
        if (cursorFlash < 1) {
            cursorFlash = 50;
            previousOutput = outputArray[cursorY][cursorX];
            outputArray[cursorY][cursorX] = previousOutput;
        }
        cursorUpdate();


        //selection
        if (a2Released && state.equals(Mode.MANUAL) && !(colors[0].equals(""))) {
            a2Pressed = false;
            a2Released = false;
            outputArray[cursorY][cursorX] = colors[0];
            if (colors[1] == "") {
                secPos = new int[]{cursorY, cursorX};
            } else {
                firstPos = new int[]{cursorY, cursorX};
            }
            colors[0] = colors[1];
            colors[1] = "";
        }

        //retrieval???? necessary???

        //confirmation
        if (b2Released && (firstPos[1] != -1 || secPos[1] != -1) && state.equals(Mode.MANUAL)) {
            b2Pressed = false;
            b2Released = false;
            confirmB = true;
        }
    }
    public void cursorUpdate() //WORKS
    {
        //left
        if (gamepad2.dpad_left) {
            leftPressed = true;
        } else if (leftPressed) {
            leftReleased = true;
        }
        //right
        if (gamepad2.dpad_right && cursorX < 13) { //12
            rightPressed = true;
        } else if (rightPressed) {
            rightReleased = true;
        }
        //down
        if (gamepad2.dpad_down && cursorY < 10) {
            downPressed = true;
        } else if (downPressed) {
            downReleased = true;
        }
        //up
        if (gamepad2.dpad_up && cursorY >= 1) {
            upPressed = true;
        } else if (upPressed) {
            upReleased = true;
        }

        //a and b
        if (gamepad2.a) {
            a2Pressed = true;
        } else if (a2Pressed) {
            a2Released = true;
        }
        //up
        if (gamepad2.b) {
            b2Pressed = true;
        } else if (b2Pressed) {
            b2Released = true;
        }

        //cursor movement
        isBoxRow();
        if (leftReleased && cursorX > 1) {
            leftPressed = false;
            leftReleased = false;
            outputArray[cursorY][cursorX] = previousOutput;
            if (cursorX - 1 > 1) {
                cursorX -= 2;
            }
        } else if (rightReleased && cursorX < 13) { //12
            rightPressed = false;
            rightReleased = false;
            outputArray[cursorY][cursorX] = previousOutput;
            if (cursorX + 1 < 13) {
                cursorX += 2;
            }
        } else if (downReleased && cursorY < 10) {
            downPressed = false;
            downReleased = false;
            outputArray[cursorY][cursorX] = previousOutput;
            cursorY++;
            if(cursorX<12) {
                if (boxRow) {
                    cursorX++;
                } else {
                    cursorX--;
                }
            }
            else {
                cursorX--;
            }
        } else if (upReleased && cursorY >= 1) {
            upPressed = false;
            upReleased = false;
            outputArray[cursorY][cursorX] = previousOutput;
            cursorY--;
            if(cursorX<12) {
                if (boxRow) {
                    cursorX++;
                } else {
                    cursorX--;
                }
            }
            else {
                cursorX--;
            }
        }
    }
    public void isBoxRow() //WORKS
    {
        if (cursorY % 2 == 0) {
            boxRow = true;
        } else {
            boxRow = false;
        }
    }
    public void printAll() //WORKS
    {
        //colors avaliable
        telemetry.addLine(String.format("                    1      2"));
        telemetry.addLine(String.format("PIXELS  - " + colors[0] + "      " + colors[1]));

        //2d array output
        String rowOut = "";
        for (int r = 0; r < outputArray.length; r++) {
            rowOut = "";
            for (int c = 0; c < outputArray[1].length; c++) {
                rowOut += outputArray[r][c];
            }
            telemetry.addData("", rowOut);
        }

        //selections queue
        if (firstPos[0] != -1 && secPos[0] != -1) {
            telemetry.addLine(String.format("QUEUE |   " + firstPos[1] + "," + firstPos[0] + " then " + secPos[1] + "," + secPos[0], null));
        } else if (firstPos[0] != -1) {
            telemetry.addLine(String.format("QUEUE |   " + firstPos[1] + "," + firstPos[0]));
        } else {
            telemetry.addLine(String.format("QUEUE |   "));

        }

        //confirmation queue
        if (confirmB && confirmA) {
            telemetry.addLine(String.format("ROBOT RUNNING"));
        } else if (confirmB) {
            telemetry.addLine(String.format("CONFIRMED PLEASE WAIT"));
        } else if (colors[1].equals("") && firstPos[0] != -1) {
            telemetry.addLine(String.format("UNCONFIRMED CHANGES"));
        } else if (colors[0].equals("") && colors[1].equals("")) {
            telemetry.addLine(String.format("NO PIXELS LOADED"));
        } else {
            telemetry.addLine(String.format("PIXELS READY TO GO"));
        }

    }
    public void makeGrid() //WORKS
    {
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
    //endregion
}