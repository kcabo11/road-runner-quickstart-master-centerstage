package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.teamcode.tuning.AutoBackstage.START_POSITION.BLUE_BACKSTAGE;
import static org.firstinspires.ftc.teamcode.tuning.AutoBackstage.START_POSITION.BLUE_STAGE;
import static org.firstinspires.ftc.teamcode.tuning.AutoBackstage.START_POSITION.RED_BACKSTAGE;
import static org.firstinspires.ftc.teamcode.tuning.AutoBackstage.START_POSITION.RED_STAGE;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.teamcode.utils.ColorDetectionProcessor;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public final class AutoBackstage extends LinearOpMode {
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
    private ColorDetectionProcessor colorDetectionProcessor;
    private ColorDetectionProcessor.StartingPosition purplePixelPath = ColorDetectionProcessor.StartingPosition.CENTER;
    public CRServo intakeLeft = null;
    public CRServo intakeRight = null;
    public void initLoop() {
        detectPurplePath();
        telemetry.addData("You selected startPosition of", startPosition.toString());
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
    }
    private void detectPurplePath() {
        purplePixelPath = colorDetectionProcessor.getPosition();
        telemetry.addData("Purple Pixel Path: ", purplePixelPath.toString());
    }

    @Override
    public void runOpMode() throws InterruptedException {

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            Pose2d beginPose = new Pose2d(14.5, 53.5, Math.toRadians(270));
            drive = new MecanumDrive(hardwareMap, beginPose);

            selectStartingPosition(); //Select stage or backstage side via the D-PAD
            initialize(); //Initialize the camera and any other devices

            while (!isStarted() && !isStopRequested())
            {
                initLoop(); //Continually find pixel path and print out the results
            }

            waitForStart();

            //OVERRIDE FOR TESTING
            startPosition = BLUE_STAGE;
            purplePixelPath = ColorDetectionProcessor.StartingPosition.RIGHT;


//          Alliance: Blue; Position: Backstage
            if (startPosition == BLUE_BACKSTAGE) {
                beginPose = new Pose2d(14.5, 53.5, Math.toRadians(270));
                telemetry.addData("Running Blue_Backstage with pixel ","");
                switch (purplePixelPath) {
                    case CENTER: {
                        telemetry.addData("CENTER", "");
                        telemetry.update();
                        Actions.runBlocking(
                                drive.actionBuilder(beginPose)
                                        .splineTo(new Vector2d(14.5, 26.5), Math.toRadians(270))
                                        .afterDisp(5, outtake_marker())
                                        .setTangent(0)
                                        .splineToLinearHeading(new Pose2d(23, 43, Math.toRadians(270)), Math.toRadians(270))
                                        .strafeTo(new Vector2d(47, 15))
                                        .build());

                        telemetry.addData("CENTER Complete", "");
                        telemetry.update();
                        break;
                    }
                    case LEFT: {
                        telemetry.addData("LEFT", "");
                        telemetry.update();
                        Actions.runBlocking(
                                drive.actionBuilder(beginPose)
                                        .splineTo(new Vector2d(23, 38), Math.toRadians(270))
                                        .afterDisp(5, outtake_marker())
                                        .setTangent(0)
                                        .splineToLinearHeading(new Pose2d(23, 43, Math.toRadians(270)), Math.toRadians(270))
                                        .strafeTo(new Vector2d(47, 15))
                                        .build());
                        telemetry.addData("LEFT Complete", "");
                        telemetry.update();
                        break;
                    }
                    case RIGHT: {
                        telemetry.addData("RIGHT", "");
                        telemetry.update();
                        Actions.runBlocking(
                                drive.actionBuilder(beginPose)
                                        .setTangent(0)
                                        .splineToConstantHeading(new Vector2d(14.5, 33), Math.toRadians(270))
                                        //.setTangent(0)
                                        .splineToLinearHeading(new Pose2d(2, 35, Math.toRadians(180)), Math.toRadians(0))
                                        //.splineToConstantHeading(new Vector2d(2, 35), Math.toRadians(180))
                                        .afterDisp(5, outtake_marker())
                                        .strafeTo(new Vector2d(47, 15))
                                        .build());
                        telemetry.addData("RIGHT Complete", "");
                        telemetry.update();
                        break;
                    }
                }

//          Alliance: Blue; Position: Frontstage
            } else if (startPosition == BLUE_STAGE) {
                beginPose = new Pose2d(-38.5, 53.5, Math.toRadians(270));
                telemetry.addData("Running Blue_Backstage with pixel ","");
                switch (purplePixelPath) {
                    case LEFT:
                        telemetry.addData("LEFT", "");
                        telemetry.update();
                        Actions.runBlocking(
                                drive.actionBuilder(beginPose)
                                        .splineTo(new Vector2d(-47, 38), Math.toRadians(270))
                                        .afterDisp(5, outtake_marker())
                                        .setTangent(0)
                                        .splineToLinearHeading(new Pose2d(-38.5, 50, Math.toRadians(270)), Math.toRadians(270))
                                        .strafeTo(new Vector2d(50, 50))
                                        .build());
                        telemetry.addData("LEFT Complete", "");
                        telemetry.update();
                        break;
                    case CENTER:
                        telemetry.addData("CENTER", "");
                        telemetry.update();
                        Actions.runBlocking(
                                drive.actionBuilder(beginPose)
                                        .splineTo(new Vector2d(-38.5, 26.5), Math.toRadians(270))
                                        .afterDisp(5, outtake_marker())
                                        .setTangent(0)
                                        .splineToLinearHeading(new Pose2d(-38.5, 50, Math.toRadians(270)), Math.toRadians(270))
                                        .strafeTo(new Vector2d(50, 50))
                                        .build());

                        telemetry.addData("CENTER Complete", "");
                        telemetry.update();
                        break;
                    case RIGHT:
                        telemetry.addData("RIGHT", "");
                        telemetry.update();
                        Actions.runBlocking(
                                drive.actionBuilder(beginPose)
                                        .setTangent(0)
                                        .splineToLinearHeading(new Pose2d(-40, 35, Math.toRadians(180)), Math.toRadians(0))
                                        //.splineToConstantHeading(new Vector2d(2, 35), Math.toRadians(180))
                                        .afterDisp(5, outtake_marker())
                                        .splineToLinearHeading(new Pose2d(-38.5, 50, Math.toRadians(270)), Math.toRadians(270))
                                        .strafeTo(new Vector2d(50, 50))
                                        .build());
                        telemetry.addData("RIGHT Complete", "");
                        telemetry.update();
                        break;
                }

//          Alliance: Red; Position: Backstage
            } else if (startPosition == RED_BACKSTAGE) {
                switch (purplePixelPath) {
                    case LEFT:
                        break;
                    case CENTER:
                        break;
                    case RIGHT:
                        break;
                }

//          Alliance: Red; Position: Frontstage
            } else if (startPosition == RED_STAGE) {
                switch (purplePixelPath) {
                    case LEFT:
                        break;
                    case CENTER:
                        break;
                    case RIGHT:
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
        return new AutoBackstage.startServoOuttake();
    }

    public void initialize(){
        //initialize other hardware as needed.
        CameraName frontCamera = hardwareMap.get(WebcamName.class, "Webcam 1");

        colorDetectionProcessor = new ColorDetectionProcessor();
        cameraPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), colorDetectionProcessor);

        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
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
                startPosition = START_POSITION.RED_BACKSTAGE;
                break;
            }
            if (gamepad1.dpad_right || gamepad2.dpad_right) {
                startPosition = START_POSITION.RED_STAGE;
                break;
            }
            telemetry.update();
        }
        telemetry.clear();
    }




}
