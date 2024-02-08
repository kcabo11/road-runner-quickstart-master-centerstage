/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.SafeTrajectoryBuilder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.utils.Processor;

// HuskyLens Integration
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

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

@Autonomous(name="Test Auto by Gyro", group="Robot")
public class RobotAutoDriveByGyro_Linear extends LinearOpMode {

    // Huskylens Integration
    private final int READ_PERIOD = 1;

    private HuskyLens huskyLens;

    /* Declare OpMode members. */
    private DcMotor         leftFront   = null;
    private DcMotor         leftBack   = null;
    private DcMotor         rightFront  = null;
    private DcMotor         rightBack  = null;
    private IMU             imu         = null;      // Control/Expansion Hub IMU
    public CRServo intakeLeft = null;
    public CRServo intakeRight = null;
    public DcMotor pixelLiftMotor = null;
    public Servo pixelPlacerServo = null;

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
    static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

    enum START_POSITION {
        //IN RELATION TO BACKBOARD
        BLUE_BACKSTAGE,
        BLUE_STAGE,
        RED_BACKSTAGE,
        RED_STAGE,
        // ????
        UNKNOWN
    }

    private VisionPortal cameraPortal;
    private static START_POSITION startPosition = START_POSITION.UNKNOWN; //WHERE WE ARE ON THE FIELD/ RED CLOSE ETC

    private Processor processor;
    private Processor.Selected purplePixelPath = Processor.Selected.LEFT;

    @Override
    public void runOpMode() {

        // Huskylens Integration ====================================================================
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        /*
         * Immediately expire so that the first time through we'll do the read.
         */
        rateLimit.expire();

        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.update();
        waitForStart();
        // ^Huskylens Integration^ ===================================================================

        // DONE: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pixelLiftMotor = hardwareMap.get(DcMotor.class, "pixelLiftMotor");
        pixelPlacerServo = hardwareMap.get(Servo.class, "pixelPlacerServo");

        pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pixelLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // DONE: reverse motor directions if needed
        //   leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");

        processor = new Processor();
        cameraPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), processor);

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
//        imu.resetYaw();

        selectStartingPosition();

        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            detectPurplePath();
            telemetry.addData("You selected startPosition of", startPosition.toString());
            telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
            telemetry.addData(" ","");
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        // Set the encoders for closed loop speed control, and reset the heading.
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //imu.resetYaw();

        // Step through each leg of the path,
        // Notes:   Reverse movement is obtained by setting a negative distance (not speed)
        //          holdHeading() is used after turns to let the heading stabilize
        //          Add a sleep(2000) after any step to keep the telemetry data visible for review

        // Huskylens Integration ====================================================================
        HuskyLens.Block[] blocks = huskyLens.blocks();
        telemetry.addData("Block count", blocks.length);
        for (int i = 0; i < blocks.length; i++) {
            telemetry.addData("Block", blocks[i].toString());
        }

        telemetry.update();
        // ^Huskylens Integration^ ===================================================================

        //OVERRIDE FOR TESTING
//        startPosition = START_POSITION.BLUE_BACKSTAGE;
//        purplePixelPath = Processor.Selected.MIDDLE;

        if (startPosition == START_POSITION.BLUE_BACKSTAGE) {
            telemetry.addData("Running Blue_Backstage with pixel ", "");
            switch (purplePixelPath) {
                case MIDDLE: {
                    driveStraight(DRIVE_SPEED, 26, 0.0);    // Drive Forward 28"
                    holdHeading(TURN_SPEED,   0.0, 2);    // Hold  0 Deg heading for .5 seconds

                    // OUTTAKE PIXEL
                    intakeLeft.setPower(.5);
                    intakeRight.setPower(-.5);
                    sleep(500);
                    // STOP OUTTAKE
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    sleep(1000);

                    driveStraight(DRIVE_SPEED, -10, 0);    // Drive Backward 10"
                    holdHeading(TURN_SPEED,   0, 2);    // Hold  0 Deg heading for 2 seconds

                    turnToHeading(TURN_SPEED, -90); // turn right 90 degrees
                    holdHeading(TURN_SPEED, -90, 2); // hold -90 degrees heading for 2 a second

                    driveStraight(DRIVE_SPEED, -38, -90);    // Drive Forward 10"
                    holdHeading(TURN_SPEED,   -90, 2);    // Hold  0 Deg heading for 2 seconds

                    // Place your pixel here:
                    // First life your pixelliftmotor
                    pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pixelLiftMotor.setTargetPosition(-484);
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
                    driveStraight(DRIVE_SPEED, 20, 0);    // Drive Forward 15"
                    holdHeading(TURN_SPEED,   0.0, 2);    // Hold  0 Deg heading for 2 seconds

                    turnToHeading(TURN_SPEED, 40); // turn left 40 degrees
                    holdHeading(TURN_SPEED, 40, 2); // hold heading for 2 a second

                    driveStraight(DRIVE_SPEED, 7, 40);    // Drive Forward 7"
                    holdHeading(TURN_SPEED,   40, 2);    // Hold  heading for 2 seconds
                    //
                    // OUTTAKE PIXEL
                    intakeLeft.setPower(.5);
                    intakeRight.setPower(-.5);
                    sleep(500);
                    // STOP OUTTAKE
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    sleep(1000);

                    driveStraight(DRIVE_SPEED, -5, 40);    // Drive Backward 10"
                    holdHeading(TURN_SPEED,   40, 2);    // Hold heading for 2 seconds

                    turnToHeading(TURN_SPEED, 90); // turn left 90 degrees
                    holdHeading(TURN_SPEED, 90, 2); // hold heading for 2 a second

                    driveStraight(DRIVE_SPEED, 29, 90);    // Drive Forward 26"
                    holdHeading(TURN_SPEED,   90, 2);    // Hold  heading for 2 seconds

                    // Place Pixel!!
                    turnToHeading(TURN_SPEED, -90); // Make a 180 degree turn
                    holdHeading(TURN_SPEED, -90, 2); // Hold heading for 2 seconds

                    // Place your pixel here:
                    // First life your pixelliftmotor
                    pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pixelLiftMotor.setTargetPosition(-484);
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
                    driveStraight(DRIVE_SPEED, 24, 0.0);    // Drive Forward 26"
                    holdHeading(TURN_SPEED,   0.0, 2);    // Hold  0 Deg heading for 2 seconds

                    turnToHeading(TURN_SPEED, -90); // turn right 90 degrees
                    holdHeading(TURN_SPEED, -90, 2); // hold -90 degrees heading for 2 a second

                    driveStraight(DRIVE_SPEED, 2, -90);    // Drive Forward 2"
                    holdHeading(TURN_SPEED,   -90, 2);    // Hold heading for 2 seconds

                    // OUTTAKE HERE
                    intakeLeft.setPower(.5);
                    intakeRight.setPower(-.5);
                    sleep(500);
                    // STOP OUTTAKE
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    sleep(1000);

                    driveStraight(DRIVE_SPEED, -37, -90);    // Drive Backward 38"
                    holdHeading(TURN_SPEED,   -90, 2);    // Hold heading for 2 seconds

                    // Place your pixel here:
                    // First life your pixelliftmotor
                    pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pixelLiftMotor.setTargetPosition(-484);
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
        else if (startPosition == START_POSITION.RED_BACKSTAGE) {
            telemetry.addData("Running Red_Backstage with pixel ", "");
            switch (purplePixelPath) {
                case MIDDLE: {
                    driveStraight(DRIVE_SPEED, 26, 0.0);    // Drive Forward 28"
                    holdHeading(TURN_SPEED,   0.0, 2);    // Hold  0 Deg heading for .5 seconds

                    // OUTTAKE PIXEL
                    intakeLeft.setPower(.5);
                    intakeRight.setPower(-.5);
                    sleep(500);
                    // STOP OUTTAKE
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    sleep(1000);

                    driveStraight(DRIVE_SPEED, -5, 0);    // Drive Backward 10"
                    holdHeading(TURN_SPEED,   0, 2);    // Hold  0 Deg heading for 2 seconds

                    turnToHeading(TURN_SPEED, 90); // turn right 90 degrees
                    holdHeading(TURN_SPEED, 90, 2); // hold -90 degrees heading for 2 a second

                    driveStraight(DRIVE_SPEED, -36, 90);    // Drive Forward 10"
                    holdHeading(TURN_SPEED,   90, 2);    // Hold  0 Deg heading for 2 seconds

//                    // Place Pixel!!
//                    turnToHeading(TURN_SPEED, 90); // Make a 180 degree turn
//                    holdHeading(TURN_SPEED, 90, 2); // Hold heading for 2 seconds

                    // Place your pixel here:
                    // First life your pixelliftmotor
                    pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pixelLiftMotor.setTargetPosition(-484);
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
                    driveStraight(DRIVE_SPEED, 26, 0.0);    // Drive Forward 26"
                    holdHeading(TURN_SPEED,   0.0, 2);    // Hold  0 Deg heading for 2 seconds

                    turnToHeading(TURN_SPEED, 90); // turn left 90 degrees
                    holdHeading(TURN_SPEED, 90, 2); // hold 90 degrees heading for 2 a second

                    driveStraight(DRIVE_SPEED, 2, 90);    // Drive Forward 2"
                    holdHeading(TURN_SPEED,   90, 2);    // Hold heading for 2 seconds

                    // OUTTAKE HERE
                    intakeLeft.setPower(.5);
                    intakeRight.setPower(-.5);
                    sleep(500);
                    // STOP OUTTAKE
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    sleep(1000);

                    driveStraight(DRIVE_SPEED, -35, 90);    // Drive Backward 38"
                    holdHeading(TURN_SPEED,   90, 2);    // Hold heading for 2 seconds

                    // Place your pixel here:
                    // First life your pixelliftmotor
                    pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pixelLiftMotor.setTargetPosition(-484);
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
                    driveStraight(DRIVE_SPEED, 16, 0);    // Drive Forward 20"
                    holdHeading(TURN_SPEED,   0.0, 2);    // Hold  0 Deg heading for 2 seconds

                    turnToHeading(TURN_SPEED, -45); // turn left 40 degrees
                    holdHeading(TURN_SPEED, -45, 2); // hold heading for 2 a second

                    driveStraight(DRIVE_SPEED, 5, -45);    // Drive Forward 5"
                    holdHeading(TURN_SPEED,   -45, 2);    // Hold  heading for 2 seconds
                    //
                    // OUTTAKE PIXEL
                    intakeLeft.setPower(.5);
                    intakeRight.setPower(-.5);
                    sleep(500);
                    // STOP OUTTAKE
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    sleep(1000);

                    driveStraight(DRIVE_SPEED, -7, -45);    // Drive Backward 10"
                    holdHeading(TURN_SPEED,   -45, 2);    // Hold heading for 2 seconds

                    turnToHeading(TURN_SPEED, -90); // turn left 90 degrees
                    holdHeading(TURN_SPEED, -90, 2); // hold heading for 2 a second

                    driveStraight(DRIVE_SPEED, 30, -90);    // Drive Forward 26"
                    holdHeading(TURN_SPEED,   -90, 2);    // Hold  heading for 2 seconds

                    turnToHeading(TURN_SPEED, 90); // make a 180 degree turn
                    holdHeading(TURN_SPEED, 90, 2); // hold heading for 2 a second

                    driveStraight(DRIVE_SPEED, -8, 90);    // Drive Forward 26"
                    holdHeading(TURN_SPEED,   90, 2);    // Hold  heading for 2 seconds

                    // Place your pixel here:
                    // First life your pixelliftmotor
                    pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pixelLiftMotor.setTargetPosition(-484);
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
            }
        }
        else if (startPosition == START_POSITION.BLUE_STAGE) {
            telemetry.addData("Running Blue_Frontstage with pixel ", "");
            switch (purplePixelPath) {
                case MIDDLE: {
                    // Requires Testing
                    driveStraight(DRIVE_SPEED, 26, 0.0);    // Drive Forward 28"
                    holdHeading(TURN_SPEED,   0.0, 2);    // Hold  0 Deg heading for .5 seconds
                    // OUTTAKE PIXEL
                    intakeLeft.setPower(.5);
                    intakeRight.setPower(-.5);
                    sleep(500);
                    // STOP OUTTAKE
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    sleep(1000);
                    driveStraight(DRIVE_SPEED, -10, 0);    // Drive Backward 10"
                    holdHeading(TURN_SPEED,   0, 2);    // Hold  0 Deg heading for 2 seconds
                    // IN CASE OF EMERGENCY:
                    // If your code interferes with your alliance, then comment the rest of this code out, and keep the upper portion
                    // That way, you can just place your purple pixel and pull back, ready for TeleOp
                    // If you are not going to interfere with your alliance, keep the following code to park
                    turnToHeading(TURN_SPEED, -90); // turn right 90 degrees
                    holdHeading(TURN_SPEED, -90, 2); // hold 90 degrees heading for 2 second
                    driveStraight(DRIVE_SPEED, 15, -90);    // Drive Forward 15"
                    holdHeading(TURN_SPEED,   -90, 2);    // Hold  0 Deg heading for 2 seconds
                    turnToHeading(TURN_SPEED, 0); // turn right 90 degrees
                    holdHeading(TURN_SPEED, 0, 2); // hold heading for 2 a second
                    driveStraight(DRIVE_SPEED, 30, 0);    // Drive Forward 30"
                    holdHeading(TURN_SPEED,   0, 2);    // Hold  0 Deg heading for 2 seconds
                    turnToHeading(TURN_SPEED, 90); // turn left 90 degrees
                    holdHeading(TURN_SPEED, 90, 2); // hold heading for 2 a second
                    driveStraight(DRIVE_SPEED, 60, 90);    // Drive Forward 60" to park in backstage
                    holdHeading(TURN_SPEED,   90, 2);    // Hold  0 Deg heading for 2 seconds

                    // Place Pixel!!
                    turnToHeading(TURN_SPEED, -90); // Make a 180 degree turn
                    holdHeading(TURN_SPEED, -90, 2); // Hold heading for 2 seconds

                    // Place your pixel here:
                    // First life your pixelliftmotor
                    pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pixelLiftMotor.setTargetPosition(-484);
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
                    // Requries Testing
                    driveStraight(DRIVE_SPEED, 15, 0);    // Drive Forward 15"
                    holdHeading(TURN_SPEED,   0.0, 2);    // Hold  0 Deg heading for 2 seconds
                    turnToHeading(TURN_SPEED, 40); // turn left 40 degrees
                    holdHeading(TURN_SPEED, 40, 2); // hold heading for 2 a second
                    driveStraight(DRIVE_SPEED, 7, 40);    // Drive Forward 7"
                    holdHeading(TURN_SPEED,   40, 2);    // Hold  heading for 2 seconds
                    // OUTTAKE PIXEL
                    intakeLeft.setPower(.5);
                    intakeRight.setPower(-.5);
                    sleep(500);
                    // STOP OUTTAKE
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    sleep(1000);
                    driveStraight(DRIVE_SPEED, -10, 40);    // Drive Backward 10"
                    holdHeading(TURN_SPEED,   40, 2);    // Hold heading for 2 seconds
                    // IN CASE OF EMERGENCY:
                    // If your code interferes with your alliance, then comment the rest of this code out, and keep the upper portion
                    // That way, you can just place your purple pixel and pull back, ready for TeleOp
                    // If you are not going to interfere with your alliance, keep the following code to park
                    turnToHeading(TURN_SPEED, 0); // turn right 90 degrees
                    holdHeading(TURN_SPEED, 0, 2); // hold heading for 2 a second
                    driveStraight(DRIVE_SPEED, 30, 0);    // Drive Forward 30"
                    holdHeading(TURN_SPEED,   0, 2);    // Hold  heading for 2 seconds
                    turnToHeading(TURN_SPEED, 90); // turn left 90 degrees
                    holdHeading(TURN_SPEED, 90, 2); // hold heading for 2 a second
                    driveStraight(DRIVE_SPEED, 50, 90);    // Drive Forward 30"
                    holdHeading(TURN_SPEED,   90, 2);    // Hold  0 Deg heading for 2 seconds

                    // Place Pixel!!
                    turnToHeading(TURN_SPEED, -90); // Make a 180 degree turn
                    holdHeading(TURN_SPEED, -90, 2); // Hold heading for 2 seconds

                    // Place your pixel here:
                    // First life your pixelliftmotor
                    pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pixelLiftMotor.setTargetPosition(-484);
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
                    // Requires Testing
                    driveStraight(DRIVE_SPEED, 26, 0.0);    // Drive Forward 26"
                    holdHeading(TURN_SPEED,   0.0, 2);    // Hold  0 Deg heading for 2 seconds
                    turnToHeading(TURN_SPEED, -90); // turn right 90 degrees
                    holdHeading(TURN_SPEED, -90, 2); // hold -90 degrees heading for 2 a second
                    driveStraight(DRIVE_SPEED, 2, -90);    // Drive Forward 2"
                    holdHeading(TURN_SPEED,   -90, 2);    // Hold heading for 2 seconds
                    // OUTTAKE HERE
                    intakeLeft.setPower(.5);
                    intakeRight.setPower(-.5);
                    sleep(500);
                    // STOP OUTTAKE
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    sleep(1000);
                    driveStraight(DRIVE_SPEED, -35, -90);    // Drive Backward 38"
                    holdHeading(TURN_SPEED,   -90, 2);    // Hold heading for 2 seconds
                    telemetry.addData("RIGHT Complete", "");
                    telemetry.update();
                    break;
                }
            }
        }
        else if (startPosition == START_POSITION.RED_STAGE) {
            telemetry.addData("Running Red_Frontstage with pixel ", "");
            switch (purplePixelPath) {
                case MIDDLE: {
                    // Requires Testing
                    driveStraight(DRIVE_SPEED, 26, 0.0);    // Drive Forward 28"
                    holdHeading(TURN_SPEED,   0.0, 2);    // Hold  0 Deg heading for .5 seconds
                    // OUTTAKE PIXEL
                    intakeLeft.setPower(.5);
                    intakeRight.setPower(-.5);
                    sleep(500);
                    // STOP OUTTAKE
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    sleep(1000);
                    driveStraight(DRIVE_SPEED, -10, 0);    // Drive Backward 10"
                    holdHeading(TURN_SPEED,   0, 2);    // Hold  0 Deg heading for 2 seconds
                    // IN CASE OF EMERGENCY:
                    // If your code interferes with your alliance, then comment the rest of this code out, and keep the upper portion
                    // That way, you can just place your purple pixel and pull back, ready for TeleOp
                    // If you are not going to interfere with your alliance, keep the following code to park
                    turnToHeading(TURN_SPEED, 90); // turn left 90 degrees
                    holdHeading(TURN_SPEED, 90, 2); // hold 90 degrees heading for 2 second
                    driveStraight(DRIVE_SPEED, 15, 90);    // Drive Forward 15"
                    holdHeading(TURN_SPEED,   90, 2);    // Hold  0 Deg heading for 2 seconds
                    turnToHeading(TURN_SPEED, 0); // turn right 90 degrees
                    holdHeading(TURN_SPEED, 0, 2); // hold heading for 2 a second
                    driveStraight(DRIVE_SPEED, 30, 0);    // Drive Forward 30"
                    holdHeading(TURN_SPEED,   0, 2);    // Hold  0 Deg heading for 2 seconds
                    turnToHeading(TURN_SPEED, -90); // turn right 90 degrees
                    holdHeading(TURN_SPEED, -90, 2); // hold heading for 2 a second
                    driveStraight(DRIVE_SPEED, 60, -90);    // Drive Forward 60" to park in backstage
                    holdHeading(TURN_SPEED,   -90, 2);    // Hold  0 Deg heading for 2 seconds

                    // Place Pixel!!
                    turnToHeading(TURN_SPEED, 90); // Make a 180 degree turn
                    holdHeading(TURN_SPEED, 90, 2); // Hold heading for 2 seconds

                    // Place your pixel here:
                    // First life your pixelliftmotor
                    pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pixelLiftMotor.setTargetPosition(-484);
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
                    // Requires Modification
                    driveStraight(DRIVE_SPEED, 20, 0);    // Drive Forward 20"
                    holdHeading(TURN_SPEED,   0.0, 2);    // Hold  0 Deg heading for 2 seconds
                    turnToHeading(TURN_SPEED, 40); // turn left 40 degrees
                    holdHeading(TURN_SPEED, 40, 2); // hold heading for 2 a second
                    driveStraight(DRIVE_SPEED, 5, 40);    // Drive Forward 5"
                    holdHeading(TURN_SPEED,   40, 2);    // Hold  heading for 2 seconds
                    //
                    // OUTTAKE PIXEL
                    intakeLeft.setPower(.5);
                    intakeRight.setPower(-.5);
                    sleep(500);
                    // STOP OUTTAKE
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    sleep(1000);
                    driveStraight(DRIVE_SPEED, -7, 40);    // Drive Backward 7"
                    holdHeading(TURN_SPEED,   40, 2);    // Hold heading for 2 seconds
                    // IN CASE OF EMERGENCY:
                    // If your code interferes with your alliance, then comment the rest of this code out, and keep the upper portion
                    // That way, you can just place your purple pixel and pull back, ready for TeleOp
                    // If you are not going to interfere with your alliance, keep the following code to park
                    turnToHeading(TURN_SPEED, 0); // turn right 90 degrees
                    holdHeading(TURN_SPEED, 0, 2); // hold heading for 2 a second
                    driveStraight(DRIVE_SPEED, 30, 0);    // Drive Forward 26"
                    holdHeading(TURN_SPEED,   0, 2);    // Hold  heading for 2 seconds
                    turnToHeading(TURN_SPEED, -90); // turn right 90 degrees
                    holdHeading(TURN_SPEED, -90, 2); // hold heading for 2 a second
                    driveStraight(DRIVE_SPEED, 45, -90);    // Drive Forward 45"
                    holdHeading(TURN_SPEED,   -90, 2);    // Hold  heading for 2 seconds

                    // Place Pixel!!
                    turnToHeading(TURN_SPEED, 90); // Make a 180 degree turn
                    holdHeading(TURN_SPEED, 90, 2); // Hold heading for 2 seconds

                    // Place your pixel here:
                    // First life your pixelliftmotor
                    pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pixelLiftMotor.setTargetPosition(-484);
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
                }
                case RIGHT: {
                    // Requries Testing
                    driveStraight(DRIVE_SPEED, 26, 0);    // Drive Forward 26"
                    holdHeading(TURN_SPEED,   0.0, 2);    // Hold  0 Deg heading for 2 seconds
                    turnToHeading(TURN_SPEED, -90); // turn right 90 degrees
                    holdHeading(TURN_SPEED, -90, 2); // hold heading for 2 a second
                    driveStraight(DRIVE_SPEED, 7, -90);    // Drive Forward 7"
                    holdHeading(TURN_SPEED,   -90, 2);    // Hold  heading for 2 seconds
                    // OUTTAKE PIXEL
                    intakeLeft.setPower(.5);
                    intakeRight.setPower(-.5);
                    sleep(500);
                    // STOP OUTTAKE
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    sleep(1000);
                    driveStraight(DRIVE_SPEED, -10, -90);    // Drive Backward -90"
                    holdHeading(TURN_SPEED,   -90, 2);    // Hold heading for 2 seconds
                    // IN CASE OF EMERGENCY:
                    // If your code interferes with your alliance, then comment the rest of this code out, and keep the upper portion
                    // That way, you can just place your purple pixel and pull back, ready for TeleOp
                    // If you are not going to interfere with your alliance, keep the following code to park
                    turnToHeading(TURN_SPEED, 0); // turn right 90 degrees
                    holdHeading(TURN_SPEED, 0, 2); // hold heading for 2 a second
                    driveStraight(DRIVE_SPEED, 30, 0);    // Drive Forward 30"
                    holdHeading(TURN_SPEED,   0, 2);    // Hold  heading for 2 seconds
                    turnToHeading(TURN_SPEED, -90); // turn right 90 degrees
                    holdHeading(TURN_SPEED, -90, 2); // hold heading for 2 a second
                    driveStraight(DRIVE_SPEED, 50, -90);    // Drive Forward 30"
                    holdHeading(TURN_SPEED,   -90, 2);    // Hold  0 Deg heading for 2 seconds

                    // Place Pixel!!
                    turnToHeading(TURN_SPEED, 90); // Make a 180 degree turn
                    holdHeading(TURN_SPEED, 90, 2); // Hold heading for 2 seconds

                    // Place your pixel here:
                    // First life your pixelliftmotor
                    pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    pixelLiftMotor.setTargetPosition(-484);
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


        sleep(1000);  // Pause to display last telemetry message.
    }


    private void detectPurplePath() {
        purplePixelPath = processor.getSelection();
        telemetry.addData("Left Saturation", processor.satRectLeft);
        telemetry.addData("Right Saturation", processor.satRectMiddle);
        telemetry.addData("Purple Pixel Path: ", purplePixelPath.toString());
        telemetry.addData("Delta Value:", (processor.satRectLeft - processor.satRectMiddle));
    }

    public void selectStartingPosition() {
        //******select start pose*****
        while (!isStopRequested()) {
            telemetry.addLine("Queen Creek Qualifier - Code Initialized");
            telemetry.addData("---------------------------------------", "");
            telemetry.addLine("Select Starting Position using DPAD Keys");
            telemetry.addData("    Blue Left   ", "(^)");
            telemetry.addData("    Blue Right ", "(v)");
            telemetry.addData("    Red Left    ", "(<)");
            telemetry.addData("    Red Right  ", "(>)");


            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                startPosition = START_POSITION.BLUE_BACKSTAGE;
                break;
            }
            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                startPosition = START_POSITION.BLUE_STAGE;
                break;
            }
            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                startPosition = START_POSITION.RED_STAGE;
                break;
            }
            if (gamepad1.dpad_right || gamepad2.dpad_right) {
                startPosition = START_POSITION.RED_BACKSTAGE;
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
}
