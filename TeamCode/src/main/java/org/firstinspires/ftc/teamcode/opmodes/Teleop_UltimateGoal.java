package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@TeleOp(name = "Teleop_UltimateGoal", group = "Teleop")
public class Teleop_UltimateGoal extends LinearOpMode
{

    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor leftBack;
    private DcMotor intake;
    private DcMotor flyWheel;
    private DcMotor arm;
    private DcMotor ramp_adjustor;
    //    private Servo intake_aid;
    private Servo claw;
    private Servo fireSelector;
    private RevBlinkinLedDriver lights;
    public ElapsedTime clawruntime = new ElapsedTime();
    public ElapsedTime fireruntime = new ElapsedTime();
    public ElapsedTime intakehelperruntime = new ElapsedTime();
    private int fire_state = 0;
    private int claw_state = 0;
    private int intake_state = 0;
    private int arm_state = 0;
    private int flywheel_state = 0;
    private double flywheelMultiplier = 1;
    // private double flywheelVoltageMultiplier = 1;
    private double flywheelVoltageSpeed = 1;

    String drivingState = "";

    private int direction = -1;
    // Setting scaling to full speed.
    private double scaleFactor = 1;
    private double scaleTurningSpeed = 1;

    private int wobble_pos = 1;

    private double theta = 22.5;
    private double delta = 45;
    private double speed = 0;
    private double calc_power;
    private double stick_directon = 0;




    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
    private void setPowers (double leftFrontSpd, double leftBackSpd, double rightFrontSpd, double rightBackSpd) {
        leftFront.setPower(leftFrontSpd);
        leftBack.setPower(leftBackSpd);
        rightFront.setPower(rightFrontSpd);
        rightBack.setPower(rightBackSpd);

    }

    @Override

    // Do not include a semicolon after the opening or closing bracket
    public void runOpMode() {;

        rightFront = hardwareMap.dcMotor.get("right_front");
        leftFront = hardwareMap.dcMotor.get("left_front");
        rightBack = hardwareMap.dcMotor.get("right_back");
        leftBack = hardwareMap.dcMotor.get("left_back");
//        intake = hardwareMap.dcMotor.get("intake");
        flyWheel = hardwareMap.dcMotor.get("fly_wheel");
        //ramp_adjustor = hardwareMap.dcMotor.get("ramp_adjuster");
        fireSelector = hardwareMap.servo.get("fire_selector");
        arm = hardwareMap.dcMotor.get("arm");
        claw = hardwareMap.servo.get("claw");
//        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {

            drivingState = "";

            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
// When the direction value is reversed this if statement inverts the addition and subtraction for turning.
// Default mode: The robot starts with the scaleTurningSpeed set to 1, scaleFactor set to 1, and direction set to forward.
            if (direction == 1) {
                final double v1 = (r * Math.cos(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v2 = (r * Math.sin(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v3 = (r * Math.sin(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v4 = (r * Math.cos(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftBack.setPower(v3);
                rightBack.setPower(v4);
            } else {
                final double v1 = (r * Math.cos(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v2 = (r * Math.sin(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v3 = (r * Math.sin(robotAngle) + (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                final double v4 = (r * Math.cos(robotAngle) - (rightX * scaleTurningSpeed)) * scaleFactor * direction;
                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftBack.setPower(v3);
                rightBack.setPower(v4);
            }

            /**
             GMAEPAD 2:
             */
//
//            if (gamepad2.right_trigger > 0){   //Intake Forwards
//                intake.setPower(1);
//            } else if (gamepad2.left_trigger > 0){    //Intake backwards
//                intake.setPower(-1);
//            } else {
//                intake.setPower(0);
//            }


            //flywheel
            if (gamepad2.right_bumper) {
                flyWheel.setPower(-1 * flywheelMultiplier); // * (-.1 * getBatteryVoltage()) + 2.25);
                //flyWheel.setPower(calc_power);
            }
            else if (gamepad2.left_bumper) {
                flyWheel.setPower(.3);
            } else {
                flyWheel.setPower(0);
            }

            //fire selector
            switch (fire_state) {
                case 0:
                    if(gamepad2.a){
                        fireruntime.reset();
                        fireSelector.setPosition(.4);
                        fire_state = 1;
                        break;
                    }
                case 1:
                    if (fireruntime.milliseconds() >= 400) {
                        fireruntime.reset();
                        fireSelector.setPosition(.6);
                        fire_state = 0;
                        break;
                    }
            }

            //Arm mover motor
            if (gamepad2.dpad_up) {
                arm.setPower(-.5);
            } else if (gamepad2.dpad_down){
                arm.setPower(.5);
            } else {
                arm.setPower(0);
            }

            //claw
            switch (claw_state) {
                case (0):
                    if (gamepad2.x) {
                        claw_state = 1;
                        claw.setPosition(wobble_pos);
                    }
                    break;
                case (1):
                    clawruntime.reset();
                    if (!gamepad2.x) {
                        claw.setPosition(-1);
                        claw_state = 0;
                        if (wobble_pos == -1) {
                            claw_state = 0;
                        } else {
                            wobble_pos = 1;
                        }
                    }
                    break;
            }



            // Flywheel power control
            switch (flywheel_state) {
                case 0:
                    if(gamepad2.right_stick_button){
                        flywheelMultiplier = .85;
//                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                        flywheel_state = 1;

                    }
                    break;
                case 1:
                    if (!gamepad2.right_stick_button) {
                        flywheel_state = 2;
                    }
                    break;
                case 2:
                    if (gamepad2.right_stick_button) {
                        flywheelMultiplier = 1;
//                        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                        flywheel_state = 3;
                    }
                    break;
                case 3:
                    if(!gamepad2.right_stick_button) {
                        flywheel_state = 0;
                    }
                    break;
            }


            //Automated arm mover to selected position

//        switch (arm_state) {
//            case 0:
//                if (gamepad2.y){
//                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    while (arm.getCurrentPosition() < 800) {
//                        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                        arm.setTargetPosition(800);
//                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        arm.setPower(.6);
//                        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    }
//                    while (claw.getPosition() < 1) // this needs to be fixed i dont have enough time
//                    claw.setPosition(1);
//                    while (arm.getCurrentPosition() < 1600) {
//                        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                        arm.setTargetPosition(1600);
//                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        arm.setPower(.6);
//                        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    }
//                    arm_state = 1;
//                }
//                break;
//            case 1:
//                if (!gamepad2.y) {
//                    arm_state = 2;
//                }
//                break;
//            case 2:
//                if (gamepad2.y) {
//                    arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                   arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    while(arm.getCurrentPosition() > -1600) {
//                       arm.setTargetPosition(-1600);
//                       arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                       arm.setPower(-.6);
//                   }
//                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                    arm_state = 3;
//                }
//                break;
//            case 3:
//                if (!gamepad2.y){
//                    arm_state = 0;
//                }
//                break;
//        }

            switch (arm_state) {
                case 0:
                    if (gamepad2.y) {
                        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        while (arm.getCurrentPosition() < 600) {
                            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            arm.setTargetPosition(600);
                            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            arm.setPower(1);
                            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        }
                        claw.setPosition(1);
                        while (arm.getCurrentPosition() < 1500) {
                            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            arm.setTargetPosition(1500);
                            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            arm.setPower(1);
                            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        }
                        arm.setPower(0);
                        arm_state = 1;
                    }
                    break;
                case 1:
                    if (!gamepad2.y) {
                        arm_state = 2;
                    }
                    break;
                case 2:
                    arm.setPower(-.09);
                    if (gamepad2.y) {
                        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        if (arm.getCurrentPosition() > -800) {
                            arm.setTargetPosition(-800);
                            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            arm.setPower(-1);
                        } else {
                            claw.setPosition(0);
                            arm.setTargetPosition(-1800);
                            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            arm.setPower(-1);
                            arm_state = 3;
                        }
                    }
                    break;
                case 3:
                    if (arm.getCurrentPosition() > -1800) {
                        claw.setPosition(0);
                        arm.setTargetPosition(-1800);
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        arm.setPower(-1);
                    } else {
                        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        arm_state = 4;
                    }

                    break;
                case 4:
                    if (!gamepad2.y) {
                        arm_state = 0;
                    }
                    break;

            }


//            telemetry.addData("H nutter", "yes");
        telemetry.addData("left_front_enc " , leftFront.getCurrentPosition());
        telemetry.addData("right_front_enc " , rightFront.getCurrentPosition());
        telemetry.addData("left_back_enc " , leftBack.getCurrentPosition());
        telemetry.addData("right_back_enc " , rightBack.getCurrentPosition());
        telemetry.addData("fire_state", fire_state);
        telemetry.addData("claw_state", claw_state);
        telemetry.addData("intake_state", intake_state);
        telemetry.addData("right back power", rightBack.getPower());
        telemetry.addData("right Front power", rightFront.getPower());
        telemetry.addData("left back power", leftBack.getPower());
        telemetry.addData("left front power", leftFront.getPower());
        telemetry.addData("claw", claw.getPosition());
        telemetry.addData("gp1 right stick y", gamepad1.right_stick_y);
        telemetry.addData("gp1 right stick x", gamepad1.right_stick_x);
        telemetry.addData("gp1 left stick y", gamepad1.left_stick_y);
        telemetry.addData("gp1 left stick x", gamepad1.left_stick_x);
        telemetry.addData("driving State", drivingState);
        telemetry.addData("computed spd", speed);
        telemetry.addData("stick_direction", stick_directon);
//        telemetry.addData("ramp_motor_enc", ramp_adjustor.getCurrentPosition());
        telemetry.addData("arm_enc", arm.getCurrentPosition());
            telemetry.addData("fire_selector position", fireSelector.getPosition());
            telemetry.addData("flywheel multiplier", flywheelMultiplier);
            telemetry.addData("flywheel_case", flywheel_state);
            telemetry.addData("right stick botton", gamepad2.right_stick_button);
            telemetry.addData("arm_case", arm_state);
            telemetry.addData("arm_enc", arm.getCurrentPosition());
//            telemetry.addData("Voltage", getBatteryVoltage());
            telemetry.addData("flywheel speed", calc_power);
//            telemetry.addData("Dead Wheel Encoder Ticks", ramp_adjustor.getCurrentPosition());
            telemetry.update();

            telemetry.addData("Hello World!","");
            telemetry.addData("We use semicolons at the end of every statement","");
            telemetry.addData("If we don't the program will not build","");
            telemetry.update();


        }
    }
}

//192.168.43.1