/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.opmodes;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;

/*
 * This OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Autonomous_Dive", group="Robot")
//@Disabled
public class Autonomous_Dive extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor leftFront   = null;
    public DcMotor leftBack   = null;
    public DcMotor  rightFront  = null;
    public DcMotor  rightBack  = null;
    public Servo DroneLauncher = null;
    public DcMotor liftDownMotor = null;
    public DcMotor liftMotor = null;
    public DcMotor pixelLiftMotor = null;
    public Servo ClawServo = null;
    public DcMotor intakeMotor = null;
    public CRServo stage1Intake = null;
    public CRServo stage2Intake = null;
    public Servo pixelLoaderLeft = null;
    public Servo pixelLoaderRight = null;
    public NormalizedColorSensor floorSensor = null;

    ColorSensor colorSensor;    // Hardware Device Object
//    ColorSensor floorSensor;    // Hardware Device Object

    DistanceSensor distanceSensor1;
    DistanceSensor distanceSensor2;

    public ModernRoboticsI2cColorSensor frontColorSensor = null;

    private DigitalChannel redLED;
    private DigitalChannel greenLED;

    double clawOffset = 0;
    double scaleTurningSpeed = .8;
    double scaleFactor = .5;
    int direction = -1;

    HardwareMap hwMap = null;
    ElapsedTime timer = new ElapsedTime();

    int planeStateMachine = 1;
    int pixelliftMotorStateMachine = 1;
    int pixelPlacerServoStateMachine = 1, scaleSpeedStateMachine = 1;
    //  WHERE WOULD INTAKE BE PLACED HERE IN THE INITIALIZATION??
    // Initialize the following:
    // Linear slide ~~ servo
    // Intake (left and right) ~~ servo
    // Pixel placer ~~ servo
    // Left pixel latch ~~ possible servo
    // Right pixel latch ~~ possible servo


    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    double v1, v2, v3, v4;


    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        boolean rightStickButtonPushed;
        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};
        float hsvValuesFloor[] = {0F,0F,0F};
        int colorSensorState = 0, pixels = 0;

//================================================================================================================
// - - - - - - - - - - - - - - - - - - - - START OF INITIALIZATION - - - - - - - - - - - - - - - - - - - - - - - -
//================================================================================================================

        // Define and Initialize Motors
        // Initialization Outline:
//           - Wheel Motor Initialization
//           - Other Motors Initialization
//           - Servo Initialization
//           - Sensor Initialization

        // Wheel Motor Initialization:
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack    = hardwareMap.get(DcMotor.class, "leftBack");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Other Motors Initialization:
        // Current Motors:
        liftDownMotor = hardwareMap.get(DcMotor.class, "liftDownMotor");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pixelLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Necessary Motors:
//         -


        // Servo Initialization:
        // Current Servos:
        pixelLiftMotor = hardwareMap.get(DcMotor.class, "pixelLiftMotor");
        ClawServo = hardwareMap.get(Servo.class, "ClawServo");
        DroneLauncher = hardwareMap.get(Servo.class, "DroneLauncher");
        stage1Intake = hardwareMap.get(CRServo.class, "intakeLeft");
        stage2Intake = hardwareMap.get(CRServo.class, "intakeRight");
        pixelLoaderLeft = hardwareMap.get(Servo.class, "pixelLoaderLeft");
        pixelLoaderRight = hardwareMap.get(Servo.class, "pixelLoaderRight");
        // Necessary Servos:
//         -

        // Sensor Initialization:
        // Current Sensors:
        colorSensor = hardwareMap.get(ColorSensor.class, "sensorColor");
        distanceSensor1 = hardwareMap.get(DistanceSensor.class, "distanceSensor1");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "distanceSensor2");

        // Necessary Sensors:
//         -

        // Send telemetry message to signify robot waiting;
        telemetry.addLine("[location] Qualifier - Autonomous Code Initialized");
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();


/*      NOTE: In the initial testing of this program, we will be utilizing the centerstage robot. The testing includes
              making sure the claw it is able to place a specimen. Rather than reconfigure and re-initialize everything
              from centerstage's program(s), we will just use the current configuration only to prove the specimen
              placing during testing. Once we prove that a specimen consistently be placed, we will transition over to
              the programmer testing robot (until we can use our current season's robot).

              With that said, the initialization of this autonomous program will be catered towards the centerstage
              robot. Once the mechanical design for this season's robot is sufficient to begin programming (or once
              we find out how many motors/servos/sensors are needed and where), we can modify this autonomous code.

              - Andrew
              September 14, 2024
 */
//================================================================================================================
// - - - - - - - - - - - - - - - - - - - - - - - START OF PROGRAM - - - - - - - - - - - - - - - - - - - - - - - -
//================================================================================================================
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // convert the RGB values to HSV values.
//        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
//        double DEFAULTHUE = hsvValues[0];

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // Specimen Placing Testing
//             NOTE: Claw is already preloaded with specimen
//            1. Lift linear slide to (set position)
            switch (pixelliftMotorStateMachine) {
                case 1: {
                    if (gamepad2.dpad_up) { //check for first button hit {
                        pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        pixelLiftMotor.setTargetPosition(-484);
                        pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        pixelLiftMotor.setPower(-.5);
                        pixelliftMotorStateMachine++;
                    }
                    break;
                }
                case 2: {
                    if (gamepad2.dpad_up == false) { //dpad button is hit again
                        pixelliftMotorStateMachine = 3;
                    }
                    break;
                }
                case 3: {
                    if (gamepad2.dpad_up) {
                        pixelLiftMotor.setTargetPosition(-986);
                        pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        pixelLiftMotor.setPower(-.5);
                        // Lift your motor to set position
                        pixelliftMotorStateMachine = 4;
                    }
                    break;
                }
                case 4: {
                    if (gamepad2.dpad_up == false) {
                        pixelliftMotorStateMachine = 5;
                    }
                    break;
                }
                case 5: {
                    if (gamepad2.dpad_up) { // dpad button is hit again
                        pixelLiftMotor.setTargetPosition(-1681);
                        pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        pixelLiftMotor.setPower(-.8);
                        // Lift your motor to set position
                        pixelliftMotorStateMachine = 6;
                    }
                    break;
                }
                case 6: {
                    if (gamepad2.dpad_up == false) {
                        pixelliftMotorStateMachine = 7;
                    }
                    break;
                }
                case 7: {
                    if (gamepad2.dpad_up) {
                        pixelLiftMotor.setTargetPosition(-2019);
                        pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        pixelLiftMotor.setPower(-.5);
                        // Lift your motor to set position
                    }
                    break;
                }
            }
//            2. Move forward (set number of inches)
//            driveStraight(DRIVE_SPEED, 20, 0);    // Drive Forward 15"
//            holdHeading(TURN_SPEED,   0.0, 1);    // Hold  0 Deg heading for 2 seconds
//            3. Bring down linear slide to (set position - enough to where the specimen clicks in)
//            4. Open claw and bring down linear slide


//            double dpad_y = 0, dpad_x = 0;
//            if (gamepad1.dpad_left) {dpad_x = -2;}
//            if (gamepad1.dpad_right) {dpad_x = 2;}
//            if (gamepad1.dpad_up) {dpad_y = -2;}
//            if (gamepad1.dpad_down) {dpad_y = 2;}
//
//            double r, robotAngle, rightX;
////            if (gamepad1.dpad_left || gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_right) {
////                r = Math.hypot(-dpad_x, dpad_y);
////                robotAngle = Math.atan2(-dpad_y, dpad_x) - Math.PI / 4;
////                rightX = 0;
////                scaleFactor = 1;
////            }
////            else {
//            r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
//            robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
//            rightX = gamepad1.right_stick_x;
//            scaleFactor = 1;
////            }
//            // Default mode: The robot starts with the scaleTurningSpeed set to 1, scaleFactor set to 1, and direction set to forward.
//            if (direction == 1) {
//                v1 = (r * Math.cos(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
//                v2 = (r * Math.sin(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
//                v3 = (r * Math.sin(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
//                v4 = (r * Math.cos(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
//                leftFront.setPower(v1);
//                rightFront.setPower(v2);
//                leftBack.setPower(v3);
//                rightBack.setPower(v4);
//            } else {
//                v1 = (r * Math.cos(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
//                v2 = (r * Math.sin(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
//                v3 = (r * Math.sin(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
//                v4 = (r * Math.cos(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
//                leftFront.setPower(v1);
//                rightFront.setPower(v2);
//                leftBack.setPower(v3);
//                rightBack.setPower(v4);
//            }




//            telemetry.addData(distanceSensor1.getDistance());
//            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            //sleep(50);
//            private void driveStraightToLine () {
//                double WHITE_THRESHOLD = 55;
//                // Some sensors allow you to set your light sensor gain for optimal sensitivity...
//                // See the SensorColor sample in this folder for how to determine the optimal gain.
//                // A gain of 15 causes a Rev Color Sensor V2 to produce an Alpha value of 1.0 at about 1.5" above the floor.
//                floorSensor.setGain(15);
//
//                // Start the robot moving forward, and then begin looking for a white line.
//                leftFront.setPower(.15);
//                leftBack.setPower(.15);
//                rightFront.setPower(.15);
//                rightBack.setPower(.15);
//
//                // run until the white line is seen OR the driver presses STOP;
//                while (opModeIsActive() && (getBrightness() < WHITE_THRESHOLD)) {
//                    sleep(5);
//                }
//
//                leftFront.setPower(0);
//                leftBack.setPower(0);
//                rightFront.setPower(0);
//                rightBack.setPower(0);
//            }
//            double getBrightness() {
//                NormalizedRGBA colors = floorSensor.getNormalizedColors();
//                telemetry.addData("Light Level (0 to 1)",  "%4.2f", colors.alpha);
//                telemetry.update();
//
//                return colors.alpha;
//            }
        }
    }
}


//================================================================================================================
// - - - - - - - - - - - - - - - - - - - - - - START OF PSEUDOCODE - - - - - - - - - - - - - - - - - - - - - - - -
//================================================================================================================

/* PseudoCode Autonomous
NOTE: All positions start with a specimen preload

=========================== RedLeft and BlueLeft =============================

1. Place specimen on high chamber
2. Grab first sample
3. Place first sample in basket
4. Grab second sample
5. Place second sample in basket
6. Park in Level 1 Ascent

NOTE: If there is only one sample available to place, then we will just grab the other sample.
(In the future we need a contingency program if our program interferes with our alliance partner)

=========================== RedLeft and BlueLeft =============================
=========================== RedRight and BlueRight ===========================

1. Place specimen on high chamber
2. Grab first colored sample
3. Place first colored sample in observation zone
4. Grab second colored sample
5. Place second colored sample in observation zone
6. Park in observation zone

NOTE: If there is only one sample available to place, then we will just grab the other sample.
(In the future we need a contingency program if our program interferes with our alliance partner)

=========================== RedRight and BlueRight ===========================
 */