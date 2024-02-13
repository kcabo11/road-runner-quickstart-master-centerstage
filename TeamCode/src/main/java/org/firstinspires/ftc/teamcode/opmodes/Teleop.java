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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
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

@TeleOp(name="Teleop", group="Robot")
public class Teleop extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor leftFront   = null;
    public DcMotor leftBack   = null;
    public DcMotor  rightFront  = null;
    public DcMotor  rightBack  = null;
    public Servo DroneLauncher = null;
    public DcMotor liftDownMotor = null;
    public DcMotor liftMotor = null;
    public DcMotor pixelLiftMotor = null;
    public Servo pixelPlacerServo = null;
    public DcMotor intakeMotor = null;
    public CRServo stage1Intake = null;
    public CRServo stage2Intake = null;
    public Servo pixelLoaderLeft = null;
    public Servo pixelLoaderRight = null;

    ColorSensor colorSensor;    // Hardware Device Object
    ColorSensor floorSensor;    // Hardware Device Object


    public ModernRoboticsI2cColorSensor frontColorSensor = null;

    private DigitalChannel redLED;
    private DigitalChannel greenLED;

    double clawOffset = 0;
    double scaleTurningSpeed = .8;
    double scaleFactor = 1;
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


        // Define and Initialize Motors
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack    = hardwareMap.get(DcMotor.class, "leftBack");
        liftDownMotor = hardwareMap.get(DcMotor.class, "liftDownMotor");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        pixelLiftMotor = hardwareMap.get(DcMotor.class, "pixelLiftMotor");
        pixelPlacerServo = hardwareMap.get(Servo.class, "pixelPlacerServo");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        DroneLauncher = hardwareMap.get(Servo.class, "DroneLauncher");

        pixelLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pixelLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        colorSensor = hardwareMap.get(ColorSensor.class, "sensorColor");
        floorSensor = hardwareMap.get(ColorSensor.class, "floorSensor");

        // Get the LED colors and touch sensor from the hardwaremap
        redLED = hardwareMap.get(DigitalChannel.class, "red");
        greenLED = hardwareMap.get(DigitalChannel.class, "green");
        // change LED mode from input to output
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
        redLED.setMode(DigitalChannel.Mode.OUTPUT);

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

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // NO NEED FOR ENCODERS!!!
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        stage1Intake = hardwareMap.get(CRServo.class, "intakeLeft");
        stage2Intake = hardwareMap.get(CRServo.class, "intakeRight");
        pixelLoaderLeft = hardwareMap.get(Servo.class, "pixelLoaderLeft");
        pixelLoaderRight = hardwareMap.get(Servo.class, "pixelLoaderRight");

//        linearSlide = hardwareMap.get(Servo.class, "linearSlide");
//        pixelPlacer = hardwareMap.get(Servo.class, "pixelPlacer");
//
//        // Possible Servos:
//        leftPixelLatch = hardwareMap.get(Servo.class, "leftPixelLatch");
//        rightPixelLatch = hardwareMap.get(Servo.class, "rightPixelLatch");


        // Send telemetry message to signify robot waiting;

        telemetry.addLine("Queen Creek Qualifier - Teleop Code Initialized");
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // convert the RGB values to HSV values.
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        double DEFAULTHUE = hsvValues[0];

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // ========================== DRIVER CONTROLLER ================================================

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            // When the direction value is reversed this if statement inverts the addition and subtraction for turning.

//            switch(scaleSpeedStateMachine) {
//                case 1: {
//                    if (gamepad1.x) {
//                        scaleSpeedStateMachine = 2;
//                        scaleFactor = 0.5;
//                    }
//                }
//                break;
//                case 2: {
//                    if (!gamepad1.x) {
//                        scaleSpeedStateMachine = 3;
//                    }
//                }
//                break;
//                case 3: {
//                    if (gamepad1.x) {
//                        scaleSpeedStateMachine = 4;
//                        scaleFactor = 1;
//                    }
//                }
//                break;
//                case 4: {
//                    if (!gamepad1.x) {
//                        scaleSpeedStateMachine = 1;
//                    }
//                }
//                break;
//            }

            int dpad_y = 0, dpad_x = 0;
            if (gamepad1.dpad_left) {dpad_x = -1;}
            if (gamepad1.dpad_right) {dpad_x = 1;}
            if (gamepad1.dpad_up) {dpad_y = 1;}
            if (gamepad1.dpad_down) {dpad_y = -1;}

            double r, robotAngle, rightX;
            if (gamepad1.dpad_left || gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_right) {
                r = Math.hypot(-dpad_x, dpad_y);
                robotAngle = Math.atan2(-dpad_y, dpad_x) - Math.PI / 4;
                rightX = 0;
                scaleFactor = .7;
            }
            else {
                r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
                robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
                rightX = gamepad1.right_stick_x;
                scaleFactor = 1;
            }
            // Default mode: The robot starts with the scaleTurningSpeed set to 1, scaleFactor set to 1, and direction set to forward.
            if (direction == 1) {
                v1 = (r * Math.cos(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                v2 = (r * Math.sin(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                v3 = (r * Math.sin(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                v4 = (r * Math.cos(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftBack.setPower(v3);
                rightBack.setPower(v4);
            } else {
                v1 = (r * Math.cos(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                v2 = (r * Math.sin(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                v3 = (r * Math.sin(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                v4 = (r * Math.cos(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftBack.setPower(v3);
                rightBack.setPower(v4);
            }

            // GM0 Code for Debugging
//            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
//            double rx = gamepad1.right_stick_x;
//
//            // Denominator is the largest motor power (absolute value) or 1
//            // This ensures all the powers maintain the same ratio,
//            // but only if at least one is out of the range [-1, 1]
//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//            double frontLeftPower = (y + x + rx) / denominator;
//            double backLeftPower = (y - x + rx) / denominator;
//            double frontRightPower = (y - x - rx) / denominator;
//            double backRightPower = (y + x - rx) / denominator;
//
//            leftFront.setPower(frontLeftPower);
//            leftBack.setPower(backLeftPower);
//            rightFront.setPower(frontRightPower);
//            rightBack.setPower(backRightPower);

            // Control of the robot lifting mechanism
            // Lift motor: motor used to lift the linear slides
            // liftDownMotor: motor used to pull the robot off the ground
            // Press y extend the lift upward
            // Press a to pull lift down
            if (gamepad2.y) {
                liftMotor.setPower(1);
            } else if (gamepad2.a) {
                liftDownMotor.setPower(-1);
                liftMotor.setPower(-.25);
            } else {
                liftMotor.setPower(0);
                liftDownMotor.setPower(0);
            }


            /******************************************
             * PLANE LAUNCHER
             *****************************************/

            // Double Tap B to launch Drone
            switch (planeStateMachine) {
                case 1: {
                    if (gamepad1.b) {//check for first button hit {
                        planeStateMachine++;
                        timer.reset();
                    }
                    break;
                }
                case 2: {
                    if (gamepad1.b) { //b is hit again
                        DroneLauncher.setPosition(-1);
                        //launch your plane
                    }
                    else if (timer.seconds() > 1) {
                        DroneLauncher.setPosition(1);
//                            go back to state one
                    }
                    break;
                }
            }

            // HALF SPEED OPTION
            // Needs to be tested
            if (gamepad1.dpad_up) {
                leftFront.setPower(v1/2);
                rightFront.setPower(v1/2);
                leftBack.setPower(v1/2);
                rightBack.setPower(v1/2);
            }
            else {
                leftFront.setPower(v1);
                rightFront.setPower(v1);
                leftBack.setPower(v1);
                rightBack.setPower(v1);
            }

            //moving frontright changes the front left value
            //front left changes the right front value
            //left back changes left back
            //right back should change the right back

//            if (gamepad1.y) // DOUBLE TAPPED!!
            // shoot airplane


            // ========================== OPERATOR CONTROLLER ===========================================

            // Extend pixel placer linear slide using dpad up
            // Retract using dpad down

            // Change pixel loading side
            // X places pixel in the left spot of the pixel placer, B places pixel in the right spot
//            if (gamepad2.x) {
//                pixelLoaderLeft.setPosition(1);
//                pixelLoaderRight.setPosition(0);
//            } else if (gamepad2.b) {
//                pixelLoaderLeft.setPosition(0);
//                pixelLoaderRight.setPosition(1);
//            }

            // Move pixel placer using the left and right bumpers
            // Right bumper places pixels, left bumper returns pixel placer to resting position



            switch (pixelPlacerServoStateMachine) {
                case 1: {
                    if (gamepad2.right_bumper) { //check for first button hit {
                        telemetry.addData("Right Bumper Active",1);
                        pixelPlacerServo.setPosition(0.75);
                        pixelPlacerServoStateMachine = 2;
                    }
                    break;
                }
                case 2: {
                    if (gamepad2.right_bumper == false) {
                        pixelPlacerServoStateMachine = 3;
                    }
                    break;
                }
                case 3: {
                    if (gamepad2.right_bumper) {
                        pixelPlacerServo.setPosition(0.9);
                        pixelPlacerServoStateMachine = 4;
                    }
                    break;
                }
                case 4: {
                    if (gamepad2.right_bumper == false) {
                        pixelPlacerServoStateMachine = 1;
                    }
                    break;
                }
            }
            if (gamepad2.left_bumper) {
                pixelPlacerServo.setPosition(0);
                pixelliftMotorStateMachine = 1;
            }

            /******************************************
             * PIXEL LIFT
             *****************************************/

            if (gamepad2.right_stick_y < -0.03) { //checks out
                pixelLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                pixelLiftMotor.setPower(-.5);
            } else if (gamepad2.right_stick_y > 0.03) {
                pixelLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                pixelLiftMotor.setPower(.4);
            }
            else {
                if (!pixelLiftMotor.isBusy()) {
                    pixelLiftMotor.setTargetPosition(pixelLiftMotor.getCurrentPosition());
                    pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    pixelLiftMotor.setPower(0.05);
                    if (pixelLiftMotor.getCurrentPosition() >0) { //added so not continuously run the motor
                        pixelLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        pixelLiftMotor.setPower(0);
                    }
                }
            }

            if (gamepad2.dpad_down){
                pixelPlacerServoStateMachine = 1; //reset the pixel placer state machine so it goes to mid on the next placement.
                if (pixelLiftMotor.getCurrentPosition() > -700) {
                    pixelPlacerServo.setPosition(0);
                    sleep(1000);
                    pixelLiftMotor.setTargetPosition(0);
                    pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    pixelLiftMotor.setPower(-.5);
                    pixelliftMotorStateMachine = 1;
                }
                else {
                    pixelLiftMotor.setTargetPosition(0);
                    pixelLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    pixelLiftMotor.setPower(-.5);
                    pixelliftMotorStateMachine = 1;
                    pixelPlacerServo.setPosition(0);
                }
            }

//          Click Right_stick_button to lift liftMotor to set position
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
                    }
                    break;
                }
            }

            /******************************************
             * INTAKE
             *****************************************/

            //Intake out
            if (gamepad1.left_trigger > 0) {
                stage1Intake.setPower(.5);
                stage2Intake.setPower(.5);
                intakeMotor.setPower(.5);
            }
            //Intake in
            else if (gamepad1.right_trigger > 0) {
                stage1Intake.setPower(-.5);
                stage2Intake.setPower(-.5);
                intakeMotor.setPower(-.5);
            } else {
//                intakeLeft.setPower(0);
//                intakeRight.setPower(0);
                intakeMotor.setPower(0);
                stage1Intake.setPower(0);
                stage2Intake.setPower(0);
                intakeMotor.setPower(0);
            }


            /******************************************
             * PIXEL COUNTER
             *****************************************/
            // convert the RGB values to HSV values.
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

            switch (colorSensorState) {
                case 0: {
                    if ((hsvValues[0] > (DEFAULTHUE * 1.1)) || (hsvValues[0] < (DEFAULTHUE * .9))) {
                        //A pixel has been seen
                        pixels++;
                        colorSensorState = 1;
                        redLED.setState(true);
                        greenLED.setState(false);
                    }
                    break;
                }
                case 1: {
                    if ((hsvValues[0] < (DEFAULTHUE * 1.1)) && (hsvValues[0] > (DEFAULTHUE * .9))) {
                        //A pixel has been seen
                        colorSensorState = 2;
                    }
                    break;
                }
                case 2: {
                    if ((hsvValues[0] > (DEFAULTHUE * 1.1)) || (hsvValues[0] < (DEFAULTHUE * .9))) {
                        pixels++;
                        colorSensorState = 3;
                        redLED.setState(false);
                        greenLED.setState(true);
                    }
                    break;
                }
                case 3: {
                    if (pixelPlacerServoStateMachine > 1) {
                        // if the pixel placer has flipped, then reset to 0;;
                        pixels = 0;
                        colorSensorState = 4;
                        redLED.setState(false);
                        greenLED.setState(false);
                    }
                    break;
                }
                case 4: { //Pixel Lift
                    if (pixelliftMotorStateMachine == 1)
                    {
                        redLED.setState(false);
                        greenLED.setState(false);
                        colorSensorState = 0;
                    }
                }
            }
            // ==================================== TELEMETRY =========================================
            // Send telemetry message to signify robot running;

            telemetry.addData("leftFront: ", leftFront.getCurrentPosition());
            telemetry.addData("leftBack: ", leftBack.getCurrentPosition());
            telemetry.addData("rightBack: ", rightBack.getCurrentPosition());
            telemetry.addData("rightFront: ", rightFront.getCurrentPosition());

            telemetry.addData("v1", v1);
            telemetry.addData("v2", v2);
            telemetry.addData("v3", v3);
            telemetry.addData("v4", v4);


            telemetry.addData("pixelLiftMotor pos: ", pixelLiftMotor.getCurrentPosition());
            telemetry.addData("pixelLiftMotor pwr: ", pixelLiftMotor.getPower());
            telemetry.addData("liftMotor pwr: ", liftMotor.getPower());
            telemetry.addData("DPAD UP", gamepad2.dpad_up);
            telemetry.addData("DPAD DOWN", gamepad2.dpad_down);
            telemetry.addData("gamepad2.b", gamepad2.b);
//            telemetry.addData("gamepad2.x", gamepad2.x);
            telemetry.addData("gamepad2.left_bumper", gamepad2.left_bumper);
            telemetry.addData("gamepad2.right_bumper", gamepad2.right_bumper);
            telemetry.addData("pixelliftMotor Encoder Position: ", pixelLiftMotor.getCurrentPosition());
            telemetry.addData("pixelliftMotor State Value: ", pixelliftMotorStateMachine);
            telemetry.addData("pixelPlacerServo State Value: ", pixelPlacerServoStateMachine);
            telemetry.addData("pixelPlacerServo Position: ", pixelPlacerServo.getPosition());
            telemetry.addData("DroneLauncher Position: ", DroneLauncher.getPosition());
            telemetry.addData("DroneLauncher State: ", planeStateMachine);
            telemetry.addData("Default Hue", DEFAULTHUE);
            telemetry.addData("number of pixels", pixels);
            //FOR TEST - CAN BE REMOVED (Floor Sensor)
            Color.RGBToHSV(floorSensor.red() * 8, floorSensor.green() * 8, floorSensor.blue() * 8, hsvValuesFloor);
            telemetry.addData("floor color sensor", (hsvValues[0]));
            telemetry.update();

            //

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}


/* PseudoCode TeleOp

1.Drive:
Left joystick tilted north - Forward-All 4 motors turn forward
Left joystick tilted south - Backward-All 4 motors turn backward
Left joystick tilted west - Left
Left joystick tilted east - Right
Left joystick tilted in a diagonal - Strafe

2.Grab Pixels:
-Intake Pixels

3.Transport Pixels:
Left joystick tilted north - Forward-All 4 motors turn forward
Left joystick tilted south - Backward-All 4 motors turn backward
Left joystick tilted west - Left
Left joystick tilted east - Right
Left joystick tilted in a diagonal - Strafe

4.Place Pixels:
-Slide plate using linear slide
-Drop pixels onto backdrop

 */