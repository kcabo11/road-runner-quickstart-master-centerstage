package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous
public final class AutoBackstage extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(14.5, 53.5, Math.toRadians(270));
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        // Center Pixel Placement
//                        .splineTo(new Vector2d(18, 25.5), Math.toRadians(270))
//                        // strafe left 29 inches
//                        .strafeTo(new Vector2d(47,25.5))

                        // Left Pixel Placement
                        // .lineToY(47)
                        .splineTo(new Vector2d(23, 38), Math.toRadians(270))
//                        .waitSeconds(1)
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(23, 43, Math.toRadians(270)), Math.toRadians(270))
//                        // strafe left 29 inches
                        .strafeTo(new Vector2d(47,15))
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


                        // MeepMeep Code here






                        .build());
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else {
            throw new RuntimeException();
        }
    }
}
