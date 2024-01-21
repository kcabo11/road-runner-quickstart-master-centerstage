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
        Pose2d beginPose = new Pose2d(14.5, 54, Math.toRadians(270));
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        //.splineTo(new Vector2d(30, 30), Math.PI / 2)
                        .splineTo(new Vector2d(14.5, 15), Math.toRadians(270))

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
