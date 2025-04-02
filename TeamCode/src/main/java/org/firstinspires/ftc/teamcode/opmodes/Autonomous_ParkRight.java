package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.utils.Processor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 *  This OpMode illustrates the concept of driving an autonomous path based on Gyro (IMU) heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code uses the Universal IMU interface so it will work with either the BNO055, or BHI260 IMU.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *  The REV Logo should be facing UP, and the USB port should be facing forward.
 *  If this is not the configuration of your REV Control Hub, then the code should be modified to reflect the correct orientation.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: This code implements the requirement of calling setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  https://ftc-docs.firstinspires.org/field-coordinate-system
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous_ParkRight", group="Robot")

public class Autonomous_ParkRight extends LinearOpMode {

    // Huskylens Integration
    private final int READ_PERIOD = 1;

    private HuskyLens huskyLens;

    /* Declare OpMode members. */
    private DigitalChannel redLED;
    private DigitalChannel greenLED;

    private DcMotor         leftFront   = null;
    private DcMotor         leftBack   = null;
    private DcMotor         rightFront  = null;
    private DcMotor         rightBack  = null;
    private IMU             imu         = null;      // Control/Expansion Hub IMU
    public CRServo intakeLeft = null;
    public CRServo intakeRight = null;
    public DcMotor pixelLiftMotor = null;
    public Servo pixelPlacerServo = null;
    public NormalizedColorSensor floorSensor = null;

    private double          headingError  = 0;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int     leftTarget    = 0;
    private int     rightTarget   = 0;


    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 145.1 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 1.889765 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.3;     // Max driving speed for better distance accuracy.
    static double     TURN_SPEED              = 0.3;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable
    double gyroAngle = 0;

    enum START_POSITION {
        //IN RELATION TO BACKBOARD
        BLUE_BACKSTAGE,
        BLUE_STAGE,
        RED_BACKSTAGE,
        RED_STAGE
    }

    private VisionPortal visionPortal;
    private static START_POSITION startPosition = START_POSITION.BLUE_BACKSTAGE; //WHERE WE ARE ON THE FIELD/ RED CLOSE ETC

    private Processor processor;
    private Processor.Selected purplePixelPath = Processor.Selected.RIGHT;
    int desiredTagId = 1;

    ColorSensor colorSensor;    // Hardware Device Object

    @Override
    public void runOpMode() {

        float hsvValues[] = {0F,0F,0F};

        // Huskylens Integration ====================================================================
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        /*
         * Immediately expire so that the first time through we'll do the read.
         */
        rateLimit.expire();
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        // ^Huskylens Integration^ ===================================================================

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        pixelLiftMotor = hardwareMap.get(DcMotor.class, "pixelLiftMotor");
        pixelPlacerServo = hardwareMap.get(Servo.class, "pixelPlacerServo");
        pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pixelLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        redLED = hardwareMap.get(DigitalChannel.class, "red");
        greenLED = hardwareMap.get(DigitalChannel.class, "green");
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        redLED.setState(false);
        greenLED.setState(true);

        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
        floorSensor = hardwareMap.get(NormalizedColorSensor.class, "floorSensor");
        colorSensor = hardwareMap.get(ColorSensor.class, "sensorColor");

        //processor = new Processor();
        //cameraPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), processor);

        initAprilTag();
        setManualExposure(6, 250);

        // DONE: make sure your config has an IMU with this name (can be BNO or BHI)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        selectStartingPosition();

        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {

            telemetry.addData("You selected startPosition of", startPosition.toString());
            telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
            telemetry.addData(" ","");
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();

            if (!huskyLens.knock()) {
                telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
            } else {
                detectPurplePathHuskylens();
            }
        }

        // Set the encoders for closed loop speed control, and reset the heading.
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu.resetYaw();

        // Step through each leg of the path,
        // Notes:   Reverse movement is obtained by setting a negative distance (not speed)
        //          holdHeading() is used after turns to let the heading stabilize
        //          Add a sleep(2000) after any step to keep the telemetry data visible for review

        //OVERRIDE FOR TESTING
//        startPosition = START_POSITION.BLUE_BACKSTAGE;
//        purplePixelPath = Processor.Selected.RIGHT;
//        desiredTagId = 2;

        if (startPosition == START_POSITION.BLUE_BACKSTAGE) {
            telemetry.addData("Running Blue_Backstage with pixel ", "");
            telemetry.update();
//            sleep(2000);
            switch (purplePixelPath) {
                case MIDDLE: {
                    desiredTagId = 1;
                    driveStraight(DRIVE_SPEED, 27, 0.0);    // Drive Forward 28"
                    holdHeading(TURN_SPEED,   0.0, 1);    // Hold  0 Deg heading for .5 seconds

                    // OUTTAKE PIXEL
                    intakeLeft.setPower(.5);
                    intakeRight.setPower(-.5);
                    sleep(500);
                    // STOP OUTTAKE
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    sleep(50);

                    driveStraight(.5, -5, 0);    // Drive Backward 10"
                    holdHeading(.5,   0, 1);    // Hold  0 Deg heading for 2 seconds

                    //overturn so the camera squarely sees the QR codes
                    turnToHeading(.5, -90); // turn left 90 degrees
                    holdHeading(.5, -90, 1); // hold -90 degrees heading for 2 a second

                    precision = 2;
                    TURN_GAIN = .3;
                    driveToAprilTag((5));

                    // Place your pixel here:
                    // First life your pixelliftmotor
                    pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pixelLiftMotor.setTargetPosition(-400);
                    pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    pixelLiftMotor.setPower(-.5);

                    // Flip your pixelplacerservo and drive back a little, then strafe out of the way.
                    pixelPlacerServo.setPosition(0.9);
                    sleep(1500);

                    driveStraight(.5, 5, getHeading());    // Drive Backward 10"
                    turnToHeading(.5, 0); // turn left 90 degrees
                    holdHeading(.5, 0, 1); // hold -90 degrees heading for 2 a second

                    // Come Back down
                    pixelPlacerServo.setPosition(0);
                    sleep(1000);
                    pixelLiftMotor.setTargetPosition(0);

                    driveStraight(.5, 15, 0);    // Drive Backward 10"
                    turnToHeading(.5, 0); // turn left 90 degrees
                    holdHeading(.5, 0, 1); // hold -90 degrees heading for 2 a second

                    telemetry.addData("CENTER", "Complete");
                    telemetry.update();
                    break;
                }
                case LEFT: {
                    desiredTagId = 1;
                    driveStraight(DRIVE_SPEED, 20, 0);    // Drive Forward 15"
                    holdHeading(TURN_SPEED,   0.0, 1);    // Hold  0 Deg heading for 2 seconds

                    turnToHeading(TURN_SPEED, 40); // turn left 40 degrees
                    holdHeading(TURN_SPEED, 40, 1); // hold heading for 2 a second
                    // If necessary, you can utilize the sensor to scan the line before you go forward and place the pixel
                    driveStraight(DRIVE_SPEED, 7, 40);    // Drive Forward 7"
                    holdHeading(TURN_SPEED,   40, 1);    // Hold  heading for 2 seconds

                    // OUTTAKE PIXEL
                    intakeLeft.setPower(.5);
                    intakeRight.setPower(-.5);
                    sleep(500);
                    // STOP OUTTAKE
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
//                    sleep(1000);

                    driveStraight(DRIVE_SPEED, -6, 40);    // Drive Backward 10"
                    holdHeading(TURN_SPEED,   40, 1);    // Hold heading for 2 seconds

                    turnToHeading(TURN_SPEED, -90); //
                    holdHeading(TURN_SPEED, -90, 1); // hold heading for 2 a second

                    driveStraight(DRIVE_SPEED, -20, -90);    // Drive Forward 26"
                    holdHeading(TURN_SPEED,   -90, .5);    // Hold  heading for 2 seconds

//                      { // Alternate Route:
//                    strafeLeft(.4,.75, true);
//                    sleep(3500);
//                    driveStraight(DRIVE_SPEED, 23, 0);
//                    holdHeading(TURN_SPEED,0,1);
//
//                    turnToHeading(TURN_SPEED,-90);
//                    holdHeading(TURN_SPEED,-90,1);
//
//                    driveStraight(DRIVE_SPEED, 3,-90);
//                    holdHeading(TURN_SPEED,-90,1);
//
//                    intakeLeft.setPower(.5);
//                    intakeRight.setPower(-.5);
//                    sleep(500);
//                    intakeLeft.setPower(0);
//                    intakeRight.setPower(0);
//                    sleep(500);
//
//                    driveToAprilTag((5));
//                    // Place Pixel After this ^
//                }

                    // ONLY DO THIS IF YOU ARE IN FRONT OF THE BACKDROP
                    // OmniDrivetoAprilTag code here
                    // This will drive you to the apriltag
                    // Then you can square up against the line in front of the backdrop here
                    // Once you square, you can move backward to a specified distance and place your pixel

                    driveToAprilTag((5));
                    strafeRight(.3, .3, true);

                    // Place Pixel!!
                    turnToHeading(TURN_SPEED, -90); // Make a 180 degree turn
                    holdHeading(TURN_SPEED, -90, 1); // Hold heading for 2 seconds


                    // Place your pixel here:
                    // First life your pixelliftmotor
                    pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pixelLiftMotor.setTargetPosition(-400);
                    pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    pixelLiftMotor.setPower(-.5);
//                    // Flip your pixelplacerservo
                    pixelPlacerServo.setPosition(0.9);
                    sleep(1000);

                    driveStraight(.5, 5, getHeading());    // Drive Backward 10"
                    turnToHeading(.5, 0); // turn left 90 degrees
                    holdHeading(.5, 0, 1); // hold -90 degrees heading for 2 a second

                    // Come Back down
                    pixelPlacerServo.setPosition(0);
                    sleep(1000);
                    pixelLiftMotor.setTargetPosition(0);

                    driveStraight(.5, 15, 0);    // Drive Backward 10"
                    turnToHeading(.5, 0); // turn left 90 degrees
                    holdHeading(.5, 0, 1); // hold -90 degrees heading for 2 a second


                    telemetry.addData("LEFT Complete", "");
                    telemetry.update();
                    break;
                }
                case RIGHT: {
                    desiredTagId = 2;
                    driveStraight(DRIVE_SPEED, 24, 0.0);    // Drive Forward 26"
                    redLED.setState(true);
                    holdHeading(TURN_SPEED,   0.0, 1);    // Hold  0 Deg heading for 2 seconds

                    // If necessary, you can utilize the sensor to scan the line before you go forward and place the pixel
                    turnToHeading(TURN_SPEED, -80); // turn right 90 degrees
                    holdHeading(TURN_SPEED, -80, 1); // hold -90 degrees heading for 2 a second

                    driveStraight(DRIVE_SPEED, 3, -80);
                    holdHeading(TURN_SPEED,   -80, 1);    // Hold heading for 2 seconds

                    // OUTTAKE HERE
                    intakeLeft.setPower(.5);
                    intakeRight.setPower(-.5);
                    sleep(500);
                    // STOP OUTTAKE
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    sleep(1000);

                    // OmniDrivetoAprilTag code here
                    // This will drive you to the apriltag
                    // Then you can square up against the line in front of the backdrop here
                    // Once you square, you can move backward to a specified distance and place your pixel
//                    driveStraight(DRIVE_SPEED, -40, -90);    // Drive Backward 38"
//                    holdHeading(TURN_SPEED,   -90, 1);    // Hold heading for 2 seconds
                    precision = 2;
                    TURN_GAIN = .3;
                    driveToAprilTag((5));

                    // Place your pixel here:
                    // First life your pixelliftmotor
                    pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pixelLiftMotor.setTargetPosition(-400);
                    pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    pixelLiftMotor.setPower(-.5);

                    // Flip your pixelplacerservo
                    pixelPlacerServo.setPosition(0.9);
                    sleep(1000);

                    driveStraight(.5, 5, getHeading());    // Drive Backward 10"
                    turnToHeading(.5, 0); // turn left 90 degrees
                    holdHeading(.5, 0, 1); // hold -90 degrees heading for 2 a second

                    // Come Back down
                    pixelPlacerServo.setPosition(0);
                    sleep(1000);
                    pixelLiftMotor.setTargetPosition(0);

                    driveStraight(.5, 15, 0);    // Drive Backward 10"
                    turnToHeading(.5, 0); // turn left 90 degrees
                    holdHeading(.5, 0, 1); // hold -90 degrees heading for 2 a second


                    telemetry.addData("RIGHT Complete", "");
                    telemetry.update();
                    break;
                }
            }
        }
        else if (startPosition == START_POSITION.RED_BACKSTAGE) {
            telemetry.addData("Running Red_Backstage with pixel ", "");
            switch (purplePixelPath) {
                case MIDDLE: {
                    desiredTagId = 4;
                    driveStraight(DRIVE_SPEED, 27, 0.0);    // Drive Forward 28"
                    holdHeading(TURN_SPEED,   0.0, 1);    // Hold  0 Deg heading for .5 seconds

                    // OUTTAKE PIXEL
                    intakeLeft.setPower(.5);
                    intakeRight.setPower(-.5);
                    sleep(500);
                    // STOP OUTTAKE
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    sleep(1000);

                    driveStraight(DRIVE_SPEED, -5, 0);    // Drive Backward 10"
                    holdHeading(TURN_SPEED,   0, 1);    // Hold  0 Deg heading for 2 seconds

                    turnToHeading(TURN_SPEED, 110); // turn left 90 degrees
                    holdHeading(TURN_SPEED, 110, 1); // hold -90 degrees heading for 2 a second

                    driveStraight(DRIVE_SPEED, -20, 110);    // Drive Forward 10"
                    holdHeading(TURN_SPEED, 110, 1); // hold -90 degrees heading for 2 a second

                    turnToHeading(TURN_SPEED, 90); // turn left 90 degrees
                    holdHeading(TURN_SPEED,   90, 1);    // Hold  0 Deg heading for 2 seconds

                    // OmniDrivetoAprilTag code here
                    // This will drive you to the apriltag
                    // Then you can square up against the line in front of the backdrop here
                    // Once you square, you can move backward to a specified distance and place your pixel
                    precision = 2;
                    TURN_GAIN = .3;
                    driveToAprilTag((5));
                    turnToHeading(TURN_SPEED, 90); // turn left 90 degrees
                    holdHeading(TURN_SPEED,   90, 1);    // Hold  0 Deg heading for 2 seconds

                    // Place your pixel here:
                    // First life your pixelliftmotor
                    pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pixelLiftMotor.setTargetPosition(-400);
                    pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    pixelLiftMotor.setPower(-.5);

                    // Flip your pixelplacerservo
                    pixelPlacerServo.setPosition(0.9);
                    sleep(1000);

                    driveStraight(.5, 5, getHeading());    // Drive Backward 10"
                    turnToHeading(.5, 0); // turn left 90 degrees
                    holdHeading(.5, 0, 1);

                    // Come Back down
                    pixelPlacerServo.setPosition(0);
                    pixelPlacerServo.setPosition(0);
                    sleep(1000);
                    pixelLiftMotor.setTargetPosition(0);

                    driveStraight(.5, -20, 0);    // Drive Backward 10"
//                    turnToHeading(.5, 0); // turn left 90 degrees
                    holdHeading(.5, 0, 1); // hold -90 degrees heading for 2 a second

                    telemetry.addData("CENTER", "Complete");
                    telemetry.update();
                    break;
                }
                case LEFT: {
                    desiredTagId = 4;
                    driveStraight(DRIVE_SPEED, 26, 0.0);    // Drive Forward 26"
                    holdHeading(TURN_SPEED,   0.0, 1);    // Hold  0 Deg heading for 2 seconds

                    turnToHeading(TURN_SPEED, 100); // turn left 90 degrees
                    holdHeading(TURN_SPEED, 100, 1); // hold 90 degrees heading for 2 a second

                    // If necessary, you can utilize the sensor to scan the line before you go forward and place the pixel
                    driveStraight(DRIVE_SPEED, 2, 100);    // Drive Forward 2"
                    holdHeading(TURN_SPEED,   100, 1);    // Hold heading for 2 seconds

                    // OUTTAKE HERE
                    intakeLeft.setPower(.5);
                    intakeRight.setPower(-.5);
                    sleep(500);
                    // STOP OUTTAKE
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    sleep(1000);

                    driveStraight(DRIVE_SPEED, -20, 100);    // Drive Backward 20"
                    holdHeading(TURN_SPEED,   100, .5);    // Hold heading for .5 seconds

                    turnToHeading(TURN_SPEED, 90); // turn left 90 degrees
                    holdHeading(TURN_SPEED, 90, 1); // hold 90 degrees heading for 2 a second

                    // OmniDrivetoAprilTag code here
                    // This will drive you to the apriltag
                    // Then you can square up against the line in front of the backdrop here
                    // Once you square, you can move backward to a specified distance and place your pixel
//                    driveStraight(DRIVE_SPEED, -35, 90);    // Drive Backward 38"
//                    holdHeading(TURN_SPEED,   90, 2);    // Hold heading for 2 seconds
                    precision = 2.5;
                    TURN_GAIN = .3;
                    driveToAprilTag(5);

                    turnToHeading(TURN_SPEED, 90); // turn left 90 degrees
                    holdHeading(TURN_SPEED, 90, 1); // hold 90 degrees heading for 2 a second

                    strafeRight(.3, .1, true);

                    // Place Pixel!!
                    turnToHeading(TURN_SPEED, 90); // Make a 180 degree turn
                    holdHeading(TURN_SPEED, 90, 1); // Hold heading for 2 seconds

                    // Place your pixel here:
                    // First life your pixelliftmotor
                    pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pixelLiftMotor.setTargetPosition(-400);
                    pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    pixelLiftMotor.setPower(-.5);

                    // Flip your pixelplacerservo
                    pixelPlacerServo.setPosition(0.9);
                    sleep(1000);


                    driveStraight(+
                            .5, 5, getHeading());    // Drive Backward 10"
                    turnToHeading(.5, 0); // turn left 90 degrees
                    holdHeading(.5, 0, 1); // hold -90 degrees heading for 2 a second

                    // Come Back down
                    pixelPlacerServo.setPosition(0);
                    pixelPlacerServo.setPosition(0);
                    sleep(1000);
                    pixelLiftMotor.setTargetPosition(0);

                    driveStraight(.5, -20, 0);    // Drive Backward 10"
                    turnToHeading(.5, 0); // turn left 90 degrees
                    holdHeading(.5, 0, 1); // hold -90 degrees heading for 2 a second

                    telemetry.addData("LEFT Complete", "");
                    telemetry.update();
                    break;
                }
                case RIGHT: {
                    desiredTagId = 5;
                    driveStraight(DRIVE_SPEED, 16, 0);    // Drive Forward 20"
                    holdHeading(TURN_SPEED,   0.0, 1);    // Hold  0 Deg heading for 2 seconds

                    turnToHeading(TURN_SPEED, -45); // turn left 40 degrees
                    holdHeading(TURN_SPEED, -45, 1); // hold heading for 2 a second

                    // If necessary, you can utilize the sensor to scan the line before you go forward and place the pixel
                    driveStraight(DRIVE_SPEED, 5, -45);    // Drive Forward 5"
                    holdHeading(TURN_SPEED,   -45, 1);    // Hold  heading for 2 seconds

                    // OUTTAKE PIXEL
                    intakeLeft.setPower(.5);
                    intakeRight.setPower(-.5);
                    sleep(500);
                    // STOP OUTTAKE
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    sleep(1000);

                    driveStraight(DRIVE_SPEED, -7, -45);    // Drive Backward 10"
                    holdHeading(TURN_SPEED,   -45, 1);    // Hold heading for 2 seconds

                    turnToHeading(TURN_SPEED, -90); // turn left 90 degrees
                    holdHeading(TURN_SPEED, -90, 1); // hold heading for 2 a second

                    driveStraight(DRIVE_SPEED, 30, -90);    // Drive Forward 26"
                    holdHeading(TURN_SPEED,   -90, 1);    // Hold  heading for 2 seconds

                    turnToHeading(TURN_SPEED, 90); // make a 180 degree turn
                    holdHeading(TURN_SPEED, 90, 1); // hold heading for 2 a second

                    driveStraight(DRIVE_SPEED, -6, 90);    // Drive Forward 26"
                    holdHeading(TURN_SPEED,   90, 1);    // Hold  heading for 2 seconds
                    // OmniDrivetoAprilTag code here
                    // This will drive you to the apriltag
                    // Then you can square up against the line in front of the backdrop here
                    // Once you square, you can move backward to a specified distance and place your pixel
                    driveToAprilTag(5);

                    // Place your pixel here:
                    // First life your pixelliftmotor
                    pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pixelLiftMotor.setTargetPosition(-400);
                    pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    pixelLiftMotor.setPower(-.5);

                    // Flip your pixelplacerservo
                    pixelPlacerServo.setPosition(0.9);
                    sleep(1000);

                    driveStraight(+
                            .5, 5, getHeading());    // Drive Backward 10"
                    turnToHeading(.5, 0); // turn left 90 degrees
                    holdHeading(.5, 0, 1); // hold -90 degrees heading for 2 a second

                    // Come Back down
                    pixelPlacerServo.setPosition(0);
                    pixelPlacerServo.setPosition(0);
                    sleep(1000);
                    pixelLiftMotor.setTargetPosition(0);

                    driveStraight(.5, -15, 0);    // Drive Backward 10"
                    turnToHeading(.5, 0); // turn left 90 degrees
                    holdHeading(.5, 0, 1); // hold -90 degrees heading for 2 a second

                    telemetry.addData("LEFT Complete", "");
                    telemetry.update();
                    break;
                }
            }
        }
        else if (startPosition == START_POSITION.BLUE_STAGE) {
            telemetry.addData("Running Blue_Frontstage with pixel ", purplePixelPath.toString());
            telemetry.update();
            switch (purplePixelPath) {
                case MIDDLE: {
                    telemetry.addData("Center Spike Mark", 1);
                    telemetry.update();
                    desiredTagId = 1;
                    driveStraight(DRIVE_SPEED, 47, 0.0);    // Drive Forward 28"
                    holdHeading(TURN_SPEED,   0.0, .5);    // Hold  0 Deg heading for .5 seconds

                    turnToHeading(.5, -180);
                    holdHeading(TURN_SPEED,   -180, 1);    // Hold  0 Deg heading for .5 seconds

                    // OUTTAKE PIXEL
                    intakeLeft.setPower(.5);
                    intakeRight.setPower(-.5);
                    sleep(500);
                    // STOP OUTTAKE
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    sleep(50);

                    driveStraight(DRIVE_SPEED, -6, 180);    // Drive Forward 28"
                    holdHeading(TURN_SPEED,   180, .5);    // Hold  0 Deg heading for .5 seconds

                    //Turn torward truss
                    turnToHeading(.5, 270);
                    holdHeading(.5,   270, .5);    // Hold  0 Deg heading for 2 seconds
                    driveStraight(.5, -72, 270);    // Drive Backward 10"
                    holdHeading(TURN_SPEED,   270, .5);    // Hold  0 Deg heading for 2 seconds

                    turnToHeading(.5, 180);
                    holdHeading(TURN_SPEED,   180, .5);    // Hold  0 Deg heading for .5 seconds

                    driveStraight(DRIVE_SPEED, 23, 180);    // Drive Backward 35"
                    holdHeading(TURN_SPEED,   180, .5);    // Hold  0 Deg heading for .5 seconds
                    turnToHeading(.5, 270);
                    holdHeading(.5,   270, .5);

                    sleep(500);
                    driveToAprilTag(5);

                    turnToHeading(TURN_SPEED, -90); // Make a 180 degree turn
                    holdHeading(TURN_SPEED, -90, 1); // Hold heading for 2 seconds

                    // Place your pixel here:
                    // First life your pixelliftmotor
                    pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pixelLiftMotor.setTargetPosition(-400);
                    pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    pixelLiftMotor.setPower(-.5);

                    // Flip your pixelplacerservo
                    pixelPlacerServo.setPosition(0.9);
                    sleep(1000);

                    // Come Back down
                    pixelPlacerServo.setPosition(0);
                    pixelPlacerServo.setPosition(0);
                    sleep(1000);
                    pixelLiftMotor.setTargetPosition(0);

                    telemetry.addData("CENTER", "Complete");
                    telemetry.update();
                    break;
                }
                case RIGHT: {
                    telemetry.addData("Right Spike Mark", 1);
                    telemetry.update();
                    TURN_SPEED = .4;

                    desiredTagId = 2;
                    driveStraight(DRIVE_SPEED, 24, 0.0);    // Drive Forward 26"
                    redLED.setState(true);
                    holdHeading(TURN_SPEED,   0.0, 1);    // Hold  0 Deg heading for 2 seconds
                    greenLED.setState(true);
                    redLED.setState(false);

                    // If necessary, you can utilize the sensor to scan the line before you go forward and place the pixel
                    turnToHeading(TURN_SPEED, 280); // turn right 90 degrees
                    holdHeading(TURN_SPEED, 280, 1); // hold -90 degrees heading for 2 a second

                    driveStraight(DRIVE_SPEED, 2, 280);
                    holdHeading(TURN_SPEED,   280, 1);    // Hold heading for 2 seconds
                    greenLED.setState(false);
                    redLED.setState(true);

                    // OUTTAKE HERE
                    intakeLeft.setPower(.5);
                    intakeRight.setPower(-.5);
                    sleep(500);
                    // STOP OUTTAKE
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    sleep(500);

                    driveStraight(DRIVE_SPEED, -5, 280);
                    holdHeading(TURN_SPEED,   280, 1);    // Hold heading for 2 seconds
                    turnToHeading(TURN_SPEED, 345); // turn right 90 degrees
                    holdHeading(TURN_SPEED, 345, 0.1); // hold -90 degrees heading for 2 a second

                    // IN CASE OF EMERGENCY:
                    // If your code interferes with your alliance, then comment the rest of this code out, and keep the upper portion
                    // That way, you can just place your purple pixel and pull back, ready for TeleOp
                    // If you are not going to interfere with your alliance, keep the following code to park

                    driveStraight(DRIVE_SPEED, 28, 345);    // Drive Forward 30"
                    holdHeading(TURN_SPEED,   345, 1);    // Hold  heading for 2 seconds

                    //Turn toward truss
                    turnToHeading(.5, 270);
                    holdHeading(.5,   270, .5);    // Hold  0 Deg heading for 2 seconds
                    driveStraight(.5, -79, 270);    // Drive Backward 10"
                    holdHeading(TURN_SPEED,   270, .5);    // Hold  0 Deg heading for 2 seconds

                    turnToHeading(.5, 0);
                    holdHeading(TURN_SPEED,   0, .5);    // Hold  0 Deg heading for .5 seconds

                    driveStraight(DRIVE_SPEED, -24, 0);    // Drive Backward 35"
                    holdHeading(TURN_SPEED,   0, .5);    // Hold  0 Deg heading for .5 seconds

                    turnToHeading(TURN_SPEED, -90); // Make a 180 degree turn
                    holdHeading(TURN_SPEED, -90, 1); // Hold heading for 2 seconds

                    // IN CASE OF EMERGENCY:
                    // If your code interferes with your alliance, then comment the rest of this code out, and keep the upper portion
                    // That way, you can just place your purple pixel and pull back, ready for TeleOp
                    // If you are not going to interfere with your alliance, keep the following code to park
                    sleep(500);
                    driveToAprilTag(5);

                    // Place Pixel!!
                    turnToHeading(TURN_SPEED, -90); // Make a 180 degree turn
                    holdHeading(TURN_SPEED, -90, 1); // Hold heading for 2 seconds

                    // Place your pixel here:
                    // First life your pixelliftmotor
                    pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pixelLiftMotor.setTargetPosition(-400);
                    pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    pixelLiftMotor.setPower(-.5);

                    // Flip your pixelplacerservo
                    pixelPlacerServo.setPosition(0.9);
                    sleep(1000);
                    driveStraight(.5, 5, getHeading());    // Drive Backward 10"
                    // Come Back down
                    pixelPlacerServo.setPosition(0);
                    pixelPlacerServo.setPosition(0);
                    sleep(1000);
                    pixelLiftMotor.setTargetPosition(0);

//                    turnToHeading(.5, 0); // turn left 90 degrees
//                    holdHeading(.5, 0, 1); // hold -90 degrees heading for 2 a second

                    telemetry.addData("RIGHT Complete", "");
                    telemetry.update();
                    break;
                }
                case LEFT: {
                    telemetry.addData("Left Spike Mark", 1);
                    telemetry.update();

                    desiredTagId = 1;
                    driveStraight(DRIVE_SPEED, 24, 0.0);    // Drive Forward 26"
                    holdHeading(TURN_SPEED,   0.0, 1);    // Hold  0 Deg heading for 2 seconds

                    // If necessary, you can utilize the sensor to scan the line before you go forward and place the pixel
                    turnToHeading(TURN_SPEED, 80); // turn right 90 degrees
                    holdHeading(TURN_SPEED, 80, 1); // hold -90 degrees heading for 2 a second

                    driveStraight(DRIVE_SPEED, 3, 80);
                    holdHeading(TURN_SPEED,   80, 1);    // Hold heading for 2 seconds

                    // OUTTAKE HERE
                    intakeLeft.setPower(.5);
                    intakeRight.setPower(-.5);
                    sleep(500);
                    // STOP OUTTAKE
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    sleep(1000);

                    driveStraight(DRIVE_SPEED, -5, 80);
                    holdHeading(TURN_SPEED,   80, 1);    // Hold heading for 2 seconds
                    turnToHeading(TURN_SPEED, 0); // turn right 90 degrees
                    holdHeading(TURN_SPEED, 0, 0); // hold -90 degrees heading for 2 a second

                    // IN CASE OF EMERGENCY:
                    // If your code interferes with your alliance, then comment the rest of this code out, and keep the upper portion
                    // That way, you can just place your purple pixel and pull back, ready for TeleOp
                    // If you are not going to interfere with your alliance, keep the following code to park

                    driveStraight(DRIVE_SPEED, 25, 0);    // Drive Forward 30"
                    holdHeading(TURN_SPEED,   0, 1);    // Hold  heading for 2 seconds

                    //Turn toward truss
                    turnToHeading(.5, -90);
                    holdHeading(.5,   -90, .5);    // Hold  0 Deg heading for 2 seconds
                    driveStraight(.5, -79, -90);    // Drive Backward 10"
                    holdHeading(TURN_SPEED,   -90, .5);    // Hold  0 Deg heading for 2 seconds

                    turnToHeading(.5, 0);
                    holdHeading(TURN_SPEED,   0, .5);    // Hold  0 Deg heading for .5 seconds

                    driveStraight(DRIVE_SPEED, -28, 0);    // Drive Backward 35"
                    holdHeading(TURN_SPEED,   0, .5);    // Hold  0 Deg heading for .5 seconds
                    turnToHeading(.5, -85);
                    holdHeading(.5,   -85, .5);

                    // IN CASE OF EMERGENCY:
                    // If your code interferes with your alliance, then comment the rest of this code out, and keep the upper portion
                    // That way, you can just place your purple pixel and pull back, ready for TeleOp
                    // If you are not going to interfere with your alliance, keep the following code to park
                    sleep(500);
                    driveToAprilTag(5);
                    strafeRight(.35, .3, true);

                    driveStraight(DRIVE_SPEED, -2,-90);
                    holdHeading(TURN_SPEED,-90,.3);

                    // Place Pixel!!
                    turnToHeading(TURN_SPEED, -90); // Make a 180 degree turn
                    holdHeading(TURN_SPEED, -90, 1); // Hold heading for 2 seconds

                    // Place your pixel here:
                    // First life your pixelliftmotor
                    pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pixelLiftMotor.setTargetPosition(-400);
                    pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    pixelLiftMotor.setPower(-.5);

                    // Flip your pixelplacerservo
                    pixelPlacerServo.setPosition(0.9);
                    sleep(1000);
                    // Come Back down
                    pixelPlacerServo.setPosition(0);
                    pixelPlacerServo.setPosition(0);
                    sleep(1000);
                    pixelLiftMotor.setTargetPosition(0);

                    telemetry.addData("RIGHT Complete", "");
                    telemetry.update();
                    break;
                }
            }
        }
        else if (startPosition == START_POSITION.RED_STAGE) {
            telemetry.addData("Running Red_Frontstage with pixel ", purplePixelPath.toString());
            telemetry.update();
            switch (purplePixelPath) {
                case MIDDLE: {
                    telemetry.addData("Center Spike Mark", 1);
                    telemetry.update();
                    desiredTagId = 4;
                    driveStraight(DRIVE_SPEED, 47, 0.0);    // Drive Forward 28"
                    holdHeading(TURN_SPEED,   0.0, .5);    // Hold  0 Deg heading for .5 seconds

                    turnToHeading(.5, 180);
                    holdHeading(TURN_SPEED,   180, 1);    // Hold  0 Deg heading for .5 seconds

                    // OUTTAKE PIXEL
                    intakeLeft.setPower(.5);
                    intakeRight.setPower(-.5);
                    sleep(500);
                    // STOP OUTTAKE
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    sleep(50);

                    driveStraight(DRIVE_SPEED, -6, 180);    // Drive Forward 28"
                    holdHeading(TURN_SPEED,   180, .5);    // Hold  0 Deg heading for .5 seconds

                    //Turn torward truss
                    turnToHeading(.5, 90);
                    holdHeading(.5,   90, .5);    // Hold  0 Deg heading for 2 seconds
                    driveStraight(.5, -75, 90);    // Drive Backward 10"
                    holdHeading(TURN_SPEED,   90, .5);    // Hold  0 Deg heading for 2 seconds

                    turnToHeading(.5, 0);
                    holdHeading(TURN_SPEED,   0, .5);    // Hold  0 Deg heading for .5 seconds

                    driveStraight(DRIVE_SPEED, -30, 0);    // Drive Backward 35"
                    holdHeading(TURN_SPEED,   0, .5);    // Hold  0 Deg heading for .5 seconds
                    turnToHeading(.5, 90);
                    holdHeading(.5,   90, .5);

                    // IN CASE OF EMERGENCY:
                    // If your code interferes with your alliance, then comment the rest of this code out, and keep the upper portion
                    // That way, you can just place your purple pixel and pull back, ready for TeleOp
                    // If you are not going to interfere with your alliance, keep the following code to park
                    sleep(500);
                    driveToAprilTag(5);

                    turnToHeading(TURN_SPEED, 90); // Make a 180 degree turn
                    holdHeading(TURN_SPEED, 90, 1); // Hold heading for 2 seconds

                    // Place your pixel here:
                    // First life your pixelliftmotor
                    pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pixelLiftMotor.setTargetPosition(-400);
                    pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    pixelLiftMotor.setPower(-.5);

                    // Flip your pixelplacerservo
                    pixelPlacerServo.setPosition(0.9);
                    sleep(1000);

                    // Come Back down
                    pixelPlacerServo.setPosition(0);
                    pixelPlacerServo.setPosition(0);
                    sleep(1000);
                    pixelLiftMotor.setTargetPosition(0);


                    telemetry.addData("CENTER", "Complete");
                    telemetry.update();
                    break;
                }
                case LEFT: {
                    telemetry.addData("Left Spike Mark", 1);
                    telemetry.update();
//                    sleep(1000);

                    desiredTagId = 4;
                    driveStraight(DRIVE_SPEED, 24, 0.0);    // Drive Forward 26"
                    redLED.setState(true);
                    holdHeading(TURN_SPEED,   0.0, 1);    // Hold  0 Deg heading for 2 seconds
                    greenLED.setState(true);
                    redLED.setState(false);

                    // If necessary, you can utilize the sensor to scan the line before you go forward and place the pixel
                    turnToHeading(TURN_SPEED, 80); // turn right 90 degrees
                    holdHeading(TURN_SPEED, 80, 1); // hold -90 degrees heading for 2 a second

                    driveStraight(DRIVE_SPEED, 2, 80);
                    holdHeading(TURN_SPEED,   80, 1);    // Hold heading for 2 seconds
                    greenLED.setState(false);
                    redLED.setState(true);

                    // OUTTAKE HERE
                    intakeLeft.setPower(.5);
                    intakeRight.setPower(-.5);
                    sleep(500);
                    // STOP OUTTAKE
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    sleep(500);

                    driveStraight(DRIVE_SPEED, -6, 80);
                    holdHeading(TURN_SPEED,   80, 1);    // Hold heading for 2 seconds
                    turnToHeading(TURN_SPEED, 15); // turn right 90 degrees
                    holdHeading(TURN_SPEED, 15, 0.1); // hold -90 degrees heading for 2 a second

                    // IN CASE OF EMERGENCY:
                    // If your code interferes with your alliance, then comment the rest of this code out, and keep the upper portion
                    // That way, you can just place your purple pixel and pull back, ready for TeleOp
                    // If you are not going to interfere with your alliance, keep the following code to park

                    driveStraight(DRIVE_SPEED, 30, 15);    // Drive Forward 30"
                    holdHeading(TURN_SPEED,   15, 1);    // Hold  heading for 2 seconds

                    //Turn toward truss
                    turnToHeading(.5, 90);
                    holdHeading(.5,   90, .5);    // Hold  0 Deg heading for 2 seconds
                    driveStraight(.5, -79, 90);    // Drive Backward 10"
                    holdHeading(TURN_SPEED,   90, .5);    // Hold  0 Deg heading for 2 seconds

                    turnToHeading(.5, 0);
                    holdHeading(TURN_SPEED,   0, .5);    // Hold  0 Deg heading for .5 seconds

                    driveStraight(DRIVE_SPEED, -20, 0);    // Drive Backward 35"
                    holdHeading(TURN_SPEED,   0, .5);    // Hold  0 Deg heading for .5 seconds
                    turnToHeading(.5, 85);
                    holdHeading(.5,   85, .5);

                    // IN CASE OF EMERGENCY:
                    // If your code interferes with your alliance, then comment the rest of this code out, and keep the upper portion
                    // That way, you can just place your purple pixel and pull back, ready for TeleOp
                    // If you are not going to interfere with your alliance, keep the following code to park
                    sleep(500);
                    driveToAprilTag(5);

                    strafeRight(.3, .3, true);

                    // Place Pixel!!
                    turnToHeading(TURN_SPEED, 90); // Make a 180 degree turn
                    holdHeading(TURN_SPEED, 90, 1); // Hold heading for 2 seconds

                    // Place your pixel here:
                    // First life your pixelliftmotor
                    pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pixelLiftMotor.setTargetPosition(-400);
                    pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    pixelLiftMotor.setPower(-.5);

                    // Flip your pixelplacerservo
                    pixelPlacerServo.setPosition(0.9);
                    sleep(1000);
                    // Come Back down
                    pixelPlacerServo.setPosition(0);
                    pixelPlacerServo.setPosition(0);
                    sleep(1000);
                    pixelLiftMotor.setTargetPosition(0);

                    telemetry.addData("LEFT Complete", "");
                    telemetry.update();
                    break;
                }
                case RIGHT: {
                    telemetry.addData("Right Spike Mark", 1);
                    telemetry.update();

                    desiredTagId = 5;
                    driveStraight(DRIVE_SPEED, 24, 0.0);    // Drive Forward 26"
                    redLED.setState(true);
                    holdHeading(TURN_SPEED,   0.0, .5);    // Hold  0 Deg heading for 2 seconds
                    greenLED.setState(true);
                    redLED.setState(false);

                    // If necessary, you can utilize the sensor to scan the line before you go forward and place the pixel
                    turnToHeading(TURN_SPEED, -80); // turn right 90 degrees
                    holdHeading(TURN_SPEED, -80, .5); // hold -90 degrees heading for 2 a second

                    driveStraight(DRIVE_SPEED, 2, -80);
                    holdHeading(TURN_SPEED,   -80, .5);    // Hold heading for 2 seconds
                    greenLED.setState(false);
                    redLED.setState(true);

                    // OUTTAKE HERE
                    intakeLeft.setPower(.5);
                    intakeRight.setPower(-.5);
                    sleep(500);
                    // STOP OUTTAKE
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    sleep(1000);

                    driveStraight(DRIVE_SPEED, -5, -80);
                    holdHeading(TURN_SPEED,   -80, .5);    // Hold heading for 2 seconds
                    turnToHeading(TURN_SPEED, 0); // turn right 90 degrees
                    holdHeading(TURN_SPEED, 0, 0); // hold -90 degrees heading for 2 a second

                    // IN CASE OF EMERGENCY:
                    // If your code interferes with your alliance, then comment the rest of this code out, and keep the upper portion
                    // That way, you can just place your purple pixel and pull back, ready for TeleOp
                    // If you are not going to interfere with your alliance, keep the following code to park

                    driveStraight(DRIVE_SPEED, 28, 0);    // Drive Forward 30"
                    holdHeading(TURN_SPEED,   0, .5);    // Hold  heading for 2 seconds

                    //Turn toward truss
                    turnToHeading(.5, 90);
                    holdHeading(.5,   90, .5);    // Hold  0 Deg heading for 2 seconds
                    driveStraight(.5, -79, 90);    // Drive Backward 10"
                    holdHeading(TURN_SPEED,   90, .5);    // Hold  0 Deg heading for 2 seconds

                    turnToHeading(.5, 0);
                    holdHeading(TURN_SPEED,   0, .5);    // Hold  0 Deg heading for .5 seconds

                    driveStraight(DRIVE_SPEED, -30, 0);    // Drive Backward 35"
                    holdHeading(TURN_SPEED,   0, .5);    // Hold  0 Deg heading for .5 seconds
                    turnToHeading(.5, 85);
                    holdHeading(.5,   85, .5);

                    // IN CASE OF EMERGENCY:
                    // If your code interferes with your alliance, then comment the rest of this code out, and keep the upper portion
                    // That way, you can just place your purple pixel and pull back, ready for TeleOp
                    // If you are not going to interfere with your alliance, keep the following code to park
                    sleep(500);
                    driveToAprilTag(5);

                    // Place Pixel!!
                    turnToHeading(TURN_SPEED, 90); // Make a 180 degree turn
                    holdHeading(TURN_SPEED, 90, 1); // Hold heading for 2 seconds

                    // Place your pixel here:
                    // First life your pixelliftmotor
                    pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pixelLiftMotor.setTargetPosition(-400);
                    pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    pixelLiftMotor.setPower(-.5);

                    // Flip your pixelplacerservo
                    pixelPlacerServo.setPosition(0.9);
                    sleep(1000);
                    // Come Back down
                    pixelPlacerServo.setPosition(0);
                    pixelPlacerServo.setPosition(0);
                    sleep(1000);
                    pixelLiftMotor.setTargetPosition(0);

                    telemetry.addData("RIGHT Complete", "");
                    telemetry.update();
                    break;
                }
            }
        }
    }

    private void detectPurplePath() {
        purplePixelPath = processor.getSelection();
        telemetry.addData("Left Saturation", processor.satRectLeft);
        telemetry.addData("Right Saturation", processor.satRectMiddle);
        telemetry.addData("Purple Pixel Path: ", purplePixelPath.toString());
        telemetry.addData("Delta Value:", (processor.satRectLeft - processor.satRectMiddle));
    }

    private void detectPurplePathHuskylens() {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        telemetry.addData("Block count", blocks.length);
        int colorID = 2;
        if ((startPosition == START_POSITION.RED_STAGE) || (startPosition == START_POSITION.RED_BACKSTAGE)) {
            colorID = 1;
        }

        if (blocks.length > 0) {
            for (int i = 0; i < blocks.length; i++) {
                if ((blocks[i].id == colorID) && (blocks[i].width > 10) && (blocks[i].height > 10)
                        && (blocks[i].width < 50) && (blocks[i].height < 70)  && (blocks[i].y < 180)) {
                    telemetry.addData("Block", blocks[i].toString());
                    telemetry.addData("", huskyLens.blocks());
                    telemetry.addData("Xvalue: ", blocks[i].x);
                    telemetry.addData("Width: ", blocks[i].width);
                    telemetry.addData("Height: ", blocks[i].height);

                    if (blocks[i].x > 170) {
                        purplePixelPath = Processor.Selected.MIDDLE;
                        desiredTagId = 2;
                        // run middle
                    } else if (blocks[i].x < 170) {
                        purplePixelPath = Processor.Selected.LEFT;
                        desiredTagId = 1;
                    } else {
                        purplePixelPath = Processor.Selected.RIGHT;
                        desiredTagId = 3;
                    }
                }
            }
        }
        else {
            purplePixelPath = Processor.Selected.RIGHT;
        }

        //telemetry.addData("Debounce Timer: ", debounceTimer.seconds());
        //telemetry.addData("Real Time Spike Reading: ", newPurplePixelPath.toString());
        telemetry.addData("Settled Spike Reading: ", purplePixelPath.toString());
    }

        public void selectStartingPosition() {
        boolean selected = false;
        while (!isStopRequested() || (selected)) {
            telemetry.addLine("State Tournament - Code Initialized");
            telemetry.addData("---------------------------------------", "");
            telemetry.addLine("Select Starting Position using DPAD Keys");
            telemetry.addData("    Blue Left   ", "(^)");
            telemetry.addData("    Blue Right ", "(v)");
            telemetry.addData("    Red Left    ", "(<)");
            telemetry.addData("    Red Right  ", "(>)");

            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                startPosition = START_POSITION.BLUE_BACKSTAGE;
                selected = true;
                break;
            }
            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                startPosition = START_POSITION.BLUE_STAGE;
                selected = true;
                break;
            }
            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                startPosition = START_POSITION.RED_STAGE;
                selected = true;
                break;
            }
            if (gamepad1.dpad_right || gamepad2.dpad_right) {
                startPosition = START_POSITION.RED_BACKSTAGE;
                selected = true;
                break;
            }
            telemetry.update();
        }

        telemetry.clear();
    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************

    /**
     *  Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the OpMode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            leftTarget = leftFront.getCurrentPosition() + moveCounts;
            rightTarget = rightFront.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftFront.setTargetPosition(leftTarget);
            leftBack.setTargetPosition(leftTarget);
            rightFront.setTargetPosition(rightTarget);
            rightBack.setTargetPosition(rightTarget);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftFront.isBusy() && rightFront.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Spin on the central axis to point in a new direction.
     *  <p>
     *  Move will stop if either of these conditions occur:
     *  <p>
     *  1) Move gets to the heading (angle)
     *  <p>
     *  2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     *  Obtain & hold a heading for a finite amount of time
     *  <p>
     *  Move will stop once the requested time has elapsed
     *  <p>
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        leftFront.setPower(leftSpeed);
        leftBack.setPower(leftSpeed);
        rightFront.setPower(rightSpeed);
        rightBack.setPower(rightSpeed);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftTarget,  rightTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      leftFront.getCurrentPosition(),
                    leftBack.getCurrentPosition());
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      rightFront.getCurrentPosition(),
                    rightBack.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L : R", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(1280, 720))
                .build();
    }

    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 4.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    double SPEED_GAIN  =  0.1;//0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    double STRAFE_GAIN =  -.2;//0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = .5;//0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    boolean stopped = false;
    private void driveToAprilTag(double duration_in_seconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        //while (opModeIsActive()) {// && (timer.seconds() < duration_in_seconds)) {
         do {
            boolean targetFound = false;
            desiredTag = null;
            double drive = 0;        // Desired forward power/speed (-1 to +1)
            double strafe = 0;        // Desired strafe power/speed (-1 to +1)
            double turn = 0;        // Desired turning power/speed (-1 to +1)

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((desiredTagId < 0) || (detection.id == desiredTagId)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Desired ID", desiredTagId);
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
//            if (gamepad1.left_bumper && targetFound) {
            if (targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            sleep(10);
            if (targetFound == false) {
                stopped = true;
                telemetry.addData("Didn't find QR Code!", 1);
            }

        } while (!stopped && opModeIsActive());
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     *
     * @return
     */

    double precision = 2.3;
    public boolean moveRobot(double x, double y, double yaw) {

        // Calculate wheel powers.
        double rightFrontPower    =  x -y -yaw;
        double leftFrontPower   =  x +y +yaw;
        double rightBackPower     =  x +y -yaw;
        double leftBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }


        // Send powers to the wheels.
        leftFront.setPower(-leftFrontPower/precision);
        rightFront.setPower(-rightFrontPower/precision);
        leftBack.setPower(-leftBackPower/precision);
        rightBack.setPower(-rightBackPower/precision);

        //Consider the robot done once all powers are 0;
        return ((-leftFrontPower/precision)<.01) &&
            ((-leftBackPower/precision)<.01) &&
            ((-rightFrontPower/precision)<.01) &&
            ((-rightBackPower/precision)<.01) &&
            ((-leftFrontPower/precision)>-.01) &&
            ((-leftBackPower/precision)>-.01) &&
            ((-rightFrontPower/precision)>-.01) &&
            ((-rightBackPower/precision)>-.01);
    }
    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    private void strafeLeft(double speed, double durationSeconds, boolean intenseMode) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (timer.seconds() < durationSeconds) {
            double kspeed = interpolateCoefficient();
            leftFront.setPower(-speed);
            leftBack.setPower(kspeed*speed);
            rightFront.setPower(speed);
            rightBack.setPower(kspeed*-speed);
            telemetry.addData("kspeed", kspeed);
            telemetry.update();
            sleep(10);
        }
    }
    private void strafeRight(double speed, double durationSeconds, boolean intenseMode) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (timer.seconds() < durationSeconds) {
            double kspeed = interpolateCoefficient();
            if (intenseMode) kspeed += .8;
            leftFront.setPower(speed);
            leftBack.setPower(kspeed*-speed);
            rightFront.setPower(-speed);
            rightBack.setPower(kspeed*speed);
            telemetry.addData("kspeed", kspeed);
            telemetry.update();
            sleep(10);
        }
    }

    public double interpolateCoefficient () {
        double voltageMin = 12.5;
        double voltageMax = 13.5;
        double coefficientMin = 1.8;
        double coefficientMax = 1.9;
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        double t = (voltage-voltageMin) / (voltageMax - voltageMin);
        double interpolatedCoefficient = coefficientMin + (1-t) * (coefficientMax - coefficientMin);
        return interpolatedCoefficient;
    }

    void moveByGyro(double heading, double duration){
        double originalHeading = Math.toRadians(getHeading());
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double maxPower = .5;
        double gain = .005;

        while (timer.seconds() < duration) {
            double error = originalHeading - getHeading();
//            double leftFrontPower = Math.sin(requestedHeading - radiansIMU  + (Math.PI / 4));
//            double rightFrontPower = Math.cos(requestedHeading - radiansIMU  + (Math.PI / 4));
//            double leftBackPower = Math.cos(requestedHeading - radiansIMU  + (Math.PI / 4));
//            double rightBackPower = Math.sin(requestedHeading - radiansIMU  + (Math.PI / 4));
            double leftFrontPower = maxPower - (error * gain);
            double rightFrontPower = maxPower + (error * gain);
            double leftBackPower = maxPower + (error * gain);
            double rightBackPower = maxPower - (error * gain);

            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            leftBackPower *= maxPower;
            leftFrontPower *= maxPower;
            rightBackPower*= maxPower;
            rightFrontPower*= maxPower;

            rightBack.setPower(-rightBackPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            leftFront.setPower(-leftFrontPower);

            telemetry.addData("rightBackPower", rightBackPower);
            telemetry.addData("rightFrontPower", rightFrontPower);
            telemetry.addData("leftBackPower", leftBackPower);
            telemetry.addData("leftFrontPower", leftFrontPower);
            telemetry.addData("requestedHeading", heading);
            telemetry.addData("imuHeading", getHeading());
            telemetry.update();
            sleep(10);
        }
    }

    private void driveStraightToLine () {
        double WHITE_THRESHOLD = 55;
        // Some sensors allow you to set your light sensor gain for optimal sensitivity...
        // See the SensorColor sample in this folder for how to determine the optimal gain.
        // A gain of 15 causes a Rev Color Sensor V2 to produce an Alpha value of 1.0 at about 1.5" above the floor.
        floorSensor.setGain(15);

        // Start the robot moving forward, and then begin looking for a white line.
        leftFront.setPower(.15);
        leftBack.setPower(.15);
        rightFront.setPower(.15);
        rightBack.setPower(.15);

        // run until the white line is seen OR the driver presses STOP;
        while (opModeIsActive() && (getBrightness() < WHITE_THRESHOLD)) {
            sleep(5);
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

    }
    // to obtain reflected light, read the normalized values from the color sensor.  Return the Alpha channel.
    double getBrightness() {
        NormalizedRGBA colors = floorSensor.getNormalizedColors();
        telemetry.addData("Light Level (0 to 1)",  "%4.2f", colors.alpha);
        telemetry.update();

        return colors.alpha;
    }
}