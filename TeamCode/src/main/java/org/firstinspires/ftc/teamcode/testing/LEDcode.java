package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class LEDcode extends LinearOpMode {
    private DigitalChannel redLED;
    private DigitalChannel greenLED;

    ElapsedTime holdTimer = new ElapsedTime();
    @Override
    public void runOpMode() {
        // Get the LED colors and touch sensor from the hardwaremap
        redLED = hardwareMap.get(DigitalChannel.class, "red");
        greenLED = hardwareMap.get(DigitalChannel.class, "green");
        //touch = hardwareMap.get(DigitalChannel.class, "touch");
        boolean state = true;
        // Wait for the play button to be pressed
        waitForStart();

        holdTimer.reset();


        // change LED mode from input to output
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
        redLED.setMode(DigitalChannel.Mode.OUTPUT);

        // Loop while the Op Mode is running
        while (opModeIsActive()) {

            if (holdTimer.seconds() > 1){
                //Touch Sensor is not pressed
                state = !state;
                redLED.setState(state);
                redLED.setState(!state);
                holdTimer.reset();

            }
            telemetry.addData("state", state);
            telemetry.update();
        }
    }
}
