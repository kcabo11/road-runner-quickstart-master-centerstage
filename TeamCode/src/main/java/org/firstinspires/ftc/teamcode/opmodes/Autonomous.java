package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.Autonomous.START_POSITION.BLUE_BACKSTAGE;
import static org.firstinspires.ftc.teamcode.opmodes.Autonomous.START_POSITION.BLUE_STAGE;
import static org.firstinspires.ftc.teamcode.opmodes.Autonomous.START_POSITION.RED_BACKSTAGE;
import static org.firstinspires.ftc.teamcode.opmodes.Autonomous.START_POSITION.RED_STAGE;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.firstinspires.ftc.teamcode.utils.Processor;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public final class Autonomous extends LinearOpMode {
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

    MecanumDrive drive;
    private Processor processor;
    private Processor.Selected purplePixelPath = Processor.Selected.LEFT;
    public CRServo intakeLeft = null;
    public CRServo intakeRight = null;
    static final double     DRIVE_SPEED             = 0.3;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    private int     leftTarget    = 0;
    private int     rightTarget   = 0;
    static final double     COUNTS_PER_MOTOR_REV    = 145.1 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 1.889765 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;

    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable\
    private double          headingError  = 0;


    public void initLoop() {
        detectPurplePath();
        telemetry.addData("You selected startPosition of", startPosition.toString());
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
    }
    private void detectPurplePath() {
        purplePixelPath = processor.getSelection();
        telemetry.addData("Left Saturation", processor.satRectLeft);
        telemetry.addData("Right Saturation", processor.satRectMiddle);
        telemetry.addData("Purple Pixel Path: ", purplePixelPath.toString());
        telemetry.addData("Delta Value:", (processor.satRectLeft - processor.satRectMiddle));
    }

    @Override
    public void runOpMode() throws InterruptedException {

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            Pose2d beginPose = new Pose2d(0, 0, Math.toRadians(0));
            drive = new MecanumDrive(hardwareMap, beginPose);

            selectStartingPosition(); //Select stage or backstage side via the D-PAD
            initialize(); //Initialize the camera and any other devices

            while (!isStarted() && !isStopRequested())
            {
                initLoop(); //Continually find pixel path and print out the results
            }

            waitForStart();

            //OVERRIDE FOR TESTING
            startPosition = BLUE_BACKSTAGE;
            purplePixelPath = Processor.Selected.MIDDLE;



//          Alliance: Blue; Position: Backstage
            if (startPosition == BLUE_BACKSTAGE) {
                beginPose = new Pose2d(12, 53.5, Math.toRadians(0));
                telemetry.addData("Running Blue_Backstage with pixel ","");
                switch (purplePixelPath) {
                    case MIDDLE: {
                        telemetry.addData("CENTER", "");
                        telemetry.update();

                        // CENTER PATH
                        driveStraight(DRIVE_SPEED, 28.0, 0.0);    // Drive Forward 28"
                        holdHeading(TURN_SPEED,   0.0, 2);    // Hold  0 Deg heading for 2 seconds

                        turnToHeading(TURN_SPEED, 90); // turn left 90 degrees
                        holdHeading(TURN_SPEED, 90, 2); // hold -90 degrees heading for 2 a second

                        driveStraight(DRIVE_SPEED, 24.0, 90);    // Drive Forward 12"
                        holdHeading(TURN_SPEED,   90, 2);    // Hold  0 Deg heading for 2 seconds

                        turnToHeading(TURN_SPEED, 0.0); // turn right 90 degrees
                        holdHeading(TURN_SPEED,   0.0, 2); // hold heading for 2 seconds
                        driveStraight(DRIVE_SPEED,12, 0.0);    // Drive to park in backdrop

                        telemetry.addData("CENTER", "Complete");
                        telemetry.update();

                        break;
                    }
                    case LEFT: {
                        telemetry.addData("LEFT", "");
                        telemetry.update();

                        // LEFT PATH
                        driveStraight(DRIVE_SPEED, 5.0, 0.0);    // Drive Forward 5"
                        holdHeading(TURN_SPEED,   0.0, 1);    // Hold  0 Deg heading for 1 seconds

                        turnToHeading(TURN_SPEED, 45); // turn left 45 degrees
                        holdHeading(TURN_SPEED, 45, 2); // hold -90 degrees heading for 2 a second

                        driveStraight(DRIVE_SPEED, 20, 45);    // Drive Forward 20"
                        holdHeading(TURN_SPEED,   45, 1);    // Hold heading for 2 seconds
                        driveStraight(DRIVE_SPEED, -20, 45);    // Drive Forward -20"
                        holdHeading(TURN_SPEED,   45, 1);    // Hold heading for 2 seconds

                        turnToHeading(TURN_SPEED, 90); // turn left 90 degrees
                        holdHeading(TURN_SPEED,   90, 2); // hold heading for 2 seconds
                        driveStraight(DRIVE_SPEED,24, 90);    // Drive forward 90"

                        turnToHeading(TURN_SPEED, 0.0); // turn right 90 degrees
                        holdHeading(TURN_SPEED, 0.0, 2); // hold heading for 2 seconds

                        driveStraight(DRIVE_SPEED, 0.0, 28); // drive forward 28"
                        holdHeading(DRIVE_SPEED, 0.0, 2); // hold heading for 2 seconds
                        turnToHeading(TURN_SPEED, 90); // turn left 90 degrees
                        holdHeading(TURN_SPEED, 90, 2); // hold heading for 2 seconds

                        telemetry.addData("LEFT Complete", "");
                        telemetry.update();
                        break;
                    }
                    case RIGHT: {
                        telemetry.addData("RIGHT", "");
                        telemetry.update();

                        // RIGHT PATH
                        driveStraight(DRIVE_SPEED, 20, 0.0);    // Drive Forward 20"
                        holdHeading(TURN_SPEED,   0.0, 1);    // Hold  0 Deg heading for 1 seconds

                        turnToHeading(TURN_SPEED, -90); // turn right 90 degrees
                        holdHeading(TURN_SPEED, 90, 2); // hold -90 degrees heading for 2 a second

                        driveStraight(DRIVE_SPEED, 5, -90);    // Drive Forward 5"
                        holdHeading(TURN_SPEED,   -90, 1);    // Hold heading for 1 second
                        driveStraight(DRIVE_SPEED, -30, -90);    // Drive Forward -30"
                        holdHeading(TURN_SPEED,   -90, 1);    // Hold heading for 2 seconds

                        turnToHeading(TURN_SPEED, 90); // turn 180 degrees
                        holdHeading(TURN_SPEED,   90, 2); // hold heading for 2 seconds

                        telemetry.addData("RIGHT Complete", "");
                        telemetry.update();
                        break;
                    }
                }

//          Alliance: Blue; Position: Frontstage
            } else if (startPosition == BLUE_STAGE) {
                beginPose = new Pose2d(-36, 53.5, Math.toRadians(270));
                telemetry.addData("Running Blue_Stage with pixel ","");
                switch (purplePixelPath) {
                    case LEFT:
                        telemetry.addData("LEFT", "");
                        telemetry.update();


                        telemetry.addData("LEFT Complete", "");
                        telemetry.update();
                        break;
                    case MIDDLE:
                        telemetry.addData("CENTER", "");
                        telemetry.update();


                        telemetry.addData("CENTER Complete", "");
                        telemetry.update();
                        break;
                    case RIGHT:
                        telemetry.addData("RIGHT", "");
                        telemetry.update();


                        telemetry.addData("RIGHT Complete", "");
                        telemetry.update();
                        break;
                }

//          Alliance: Red; Position: Backstage
            } else if (startPosition == RED_BACKSTAGE) {
                beginPose = new Pose2d(12, -53.5, Math.toRadians(90));
                telemetry.addData("Running Red_Backstage with pixel ","");
                switch (purplePixelPath) {
                    case LEFT:
                        telemetry.addData("LEFT", "");
                        telemetry.update();


                        telemetry.addData("LEFT Complete", "");
                        telemetry.update();
                        break;
                    case MIDDLE:
                        telemetry.addData("CENTER", "");
                        telemetry.update();


                        telemetry.addData("CENTER Complete", "");
                        telemetry.update();
                        break;
                    case RIGHT:
                        telemetry.addData("RIGHT", "");
                        telemetry.update();


                        telemetry.addData("RIGHT Complete", "");
                        telemetry.update();
                        break;
                }

//          Alliance: Red; Position: Frontstage
            } else if (startPosition == RED_STAGE) {
                beginPose = new Pose2d(-36, -53.5, Math.toRadians(90));
                telemetry.addData("Running Red_Stage with pixel ","");
                switch (purplePixelPath) {
                    case LEFT:
                        telemetry.addData("LEFT", "");
                        telemetry.update();


                        telemetry.addData("LEFT Complete", "");
                        telemetry.update();
                        break;
                    case MIDDLE:
                        telemetry.addData("CENTER", "");
                        telemetry.update();


                        telemetry.addData("CENTER Complete", "");
                        telemetry.update();
                        break;
                    case RIGHT:
                        telemetry.addData("RIGHT", "");
                        telemetry.update();


                        telemetry.addData("RIGHT Complete", "");
                        telemetry.update();
                        break;
                }
            }


//                Actions.runBlocking(
//                    drive.actionBuilder(beginPose)
//                            // Center Pixel Placement
//    //                        .splineTo(new Vector2d(18, 25.5), Math.toRadians(270))
//    //                        // strafe left 29 inches
//    //                        .strafeTo(new Vector2d(47,25.5))
//
//                            // Left Pixel Placement
//                            // .lineToY(47)
//                            .splineTo(new Vector2d(23, 38), Math.toRadians(270))
//    //                        .waitSeconds(1)
//                            .setTangent(0)
//                            .splineToLinearHeading(new Pose2d(23, 43, Math.toRadians(270)), Math.toRadians(270))
//    //                        // strafe left 29 inches
//                            .strafeTo(new Vector2d(47,15))
    //
    //                        // Right Pixel Placement
    //                        .splineTo(new Vector2d(1, 38), Math.toRadians(270))
    //                        // strafe left 29 inches
    //                        .strafeTo(new Vector2d(47,25.5))


                            /* Placing pixel code here
                            .waitSeconds(2)
                            .setTangent(0)
                            .splineTo(new Vector2d(36, 34.5), Math.toRadians(180))
                            */


                            // CENTER POS
                           // .lineToY(-27)
                            //.back(2)


                            // sleep
                            //.splineToSplineHeading(new Pose2d(45, -35, Math.toRadians(180)), Math.toRadians(0))

                            // LEFT POS
                            // .strafeRight(10)
                            // .forward(5)
                           //  .splineToSplineHeading(new Pose2d(25, -35, Math.toRadians(180)), Math.toRadians(0))
                            // .forward(13)

                            // sleep
                            // .back(34)

                            // RIGHT POS
                            // .forward(30)
                            // .strafeLeft(2)
                            //.turn(Math.toRadians(-90))
                            // .forward(3)
                            // .back(2)
                            // .strafeRight(23)
                            // .forward(20)
                            // .splineToSplineHeading(new Pose2d(48, -35, Math.toRadians(180)), Math.toRadians(0))

                            // .strafeLeft(22)

                            // .forward(47)
                            // .turn(Math.toRadians(90))
                            // .forward(3)
                            // .back(5)
                            // .turn(Math.toRadians(180))
                            // .forward(115)
                            // .forward(

//
//                        .build());
        } else {
            throw new RuntimeException();
        }
    }

    public class startServoOuttake implements Action {
        private boolean initialized = false;

        @ Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeLeft.setPower(.5);
            intakeRight.setPower(-.5);
            return true;
        }
    }

    public Action outtake_marker() {
        return new Autonomous.startServoOuttake();
    }

    public class stopServoOuttake implements Action {
        private boolean initialized = false;

        @ Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
            return true;
        }
    }

    public Action stop_outtake() {
        return new Autonomous.stopServoOuttake();
    }

    public void initialize(){
        //initialize other hardware as needed.
        CameraName frontCamera = hardwareMap.get(WebcamName.class, "Webcam 1");

        processor = new Processor();
        cameraPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), processor);

        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
    }

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
            leftTarget = drive.leftFront.getCurrentPosition() + moveCounts;
            rightTarget = drive.rightFront.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            drive.leftFront.setTargetPosition(leftTarget);
            drive.leftBack.setTargetPosition(leftTarget);
            drive.rightFront.setTargetPosition(rightTarget);
            drive.rightBack.setTargetPosition(rightTarget);

            drive.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (drive.leftFront.isBusy() && drive.rightFront.isBusy())) {

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
            drive.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drive.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drive.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drive.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
     * @param drivespeed_arg forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    private void moveRobot(double drivespeed_arg, double turn) {
        driveSpeed = drivespeed_arg;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drivespeed_arg - turn;
        rightSpeed = drivespeed_arg + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        drive.leftFront.setPower(leftSpeed);
        drive.leftBack.setPower(leftSpeed);
        drive.rightFront.setPower(rightSpeed);
        drive.rightBack.setPower(rightSpeed);
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
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      drive.leftFront.getCurrentPosition(),
                    drive.leftBack.getCurrentPosition());
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      drive.rightFront.getCurrentPosition(),
                    drive.rightBack.getCurrentPosition());
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
        YawPitchRollAngles orientation = drive.imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }


    public void selectStartingPosition() {
        //******select start pose*****
        while (!isStopRequested()) {
            telemetry.addLine("Glendale Qualifier - Code Initialized");
            telemetry.addData("---------------------------------------", "");
            telemetry.addLine("Select Starting Position using DPAD Keys");
            telemetry.addData("    Blue Left   ", "(^)");
            telemetry.addData("    Blue Right ", "(v)");
            telemetry.addData("    Red Left    ", "(<)");
            telemetry.addData("    Red Right  ", "(>)");


            if (gamepad1.dpad_up || gamepad2.dpad_up) {
                startPosition = BLUE_BACKSTAGE;
                break;
            }
            if (gamepad1.dpad_down || gamepad2.dpad_down) {
                startPosition = BLUE_STAGE;
                break;
            }
            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                startPosition = RED_STAGE;
                break;
            }
            if (gamepad1.dpad_right || gamepad2.dpad_right) {
                startPosition = RED_BACKSTAGE;
                break;
            }
            telemetry.update();
        }
        telemetry.clear();
    }
}