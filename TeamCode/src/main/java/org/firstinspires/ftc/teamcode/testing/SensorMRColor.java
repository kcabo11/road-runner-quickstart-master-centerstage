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

package org.firstinspires.ftc.teamcode.testing;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.MovingAverage;

/*
 *
 * This OpMode that shows how to use
 * a Modern Robotics Color Sensor.
 *
 * The OpMode assumes that the color sensor
 * is configured with a name of "sensor_color".
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@TeleOp(name = "Sensor: MR Color", group = "Sensor")
//@Disabled
public class SensorMRColor extends LinearOpMode {

  ColorSensor colorSensor;    // Hardware Device Object
  private MovingAverage movingAverage1 = new MovingAverage(10);
  public DistanceSensor distanceSensor1;
  int distanceSensorStateMachine = 1;
  ElapsedTime timer = new ElapsedTime();

  @Override
  public void runOpMode() {

    distanceSensor1 = hardwareMap.get(DistanceSensor.class, "distanceSensor1");
    Rev2mDistanceSensor sensorTimeOfFlight1 = (Rev2mDistanceSensor) distanceSensor1;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // get a reference to the RelativeLayout so we can change the background
    // color of the Robot Controller app to match the hue detected by the RGB sensor.
    int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
    final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

    // bPrevState and bCurrState represent the previous and current state of the button.
    boolean bPrevState = false;
    boolean bCurrState = false;

    // bLedOn represents the state of the LED.
    boolean bLedOn = true;

    int colorSensorState = 0, colorCountpixels = 0, distanceCountpixels = 0;
    // get a reference to our ColorSensor object.
    colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

    // Set the LED in the beginning
    colorSensor.enableLed(bLedOn);

    // wait for the start button to be pressed.
    waitForStart();

    // convert the RGB values to HSV values.
    Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
    double DEFAULTHUE = hsvValues[0];

    // while the OpMode is active, loop and read the RGB data.
    // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
    while (opModeIsActive()) {

//    TESTING OBJECTIVE:
//    Develop code for color and distance sensor to simultaneously count pixels as they pass by
// ========================================================================================================================================
      // Distance sensor code:
      movingAverage1.add((float)(distanceSensor1.getDistance(DistanceUnit.CM)));
      telemetry.addData("movingAverage1: ", movingAverage1.getAverage());

      switch (distanceSensorStateMachine) {
                case 1: {
                    if (movingAverage1.getAverage() < 5) { //check for distance under __ cm (whatever the distance is between the sensor and pixel)
                        distanceSensorStateMachine++;
                        timer.reset();
                        distanceCountpixels++;
                        // IF YOU SEE IT GO TO CASE 2
                    }
                    break;
                }
                case 2: {
                  if (timer.seconds() > 1.5) {
                    distanceSensorStateMachine = 1;
                        // WAIT FOR HALF A SECOND
                    }
                    break;
                }
            }

            // Distance sensor task for next time:
      /*
            Modify the code to where the distance sensor can recognize a 10% change with any marginal threshold.
            Basically, it should be able to recognize a change in distance no matter how far or close it starts
       */
// ========================================================================================================================================
      // Color sensor code:

      // check the status of the x button on either gamepad.
      bCurrState = gamepad1.x;

      // check for button state transitions.
      if (bCurrState && (bCurrState != bPrevState))  {

        // button is transitioning to a pressed state. So Toggle LED
        bLedOn = !bLedOn;
        colorSensor.enableLed(bLedOn);
      }

      // update previous state variable.
      bPrevState = bCurrState;

      // convert the RGB values to HSV values.
      Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

      switch (colorSensorState) {
        case 0: {
          if ((hsvValues[0] > (DEFAULTHUE * 1.1)) || (hsvValues[0] < (DEFAULTHUE * .9))) {
            //A pixel has been seen
            colorCountpixels++;
            colorSensorState = 1;
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
            colorCountpixels++;
            colorSensorState = 3;
          }
          break;
        }
        case 3: {
          if ((hsvValues[0] < (DEFAULTHUE * 1.1)) && (hsvValues[0] > (DEFAULTHUE * .9))) {
            // if the pixel placer has flipped, then reset to 0;;
            // pixels = 0;
            colorSensorState = 0;
          }
          break;
        }
      }



      // send the info back to driver station using telemetry function.
      // Color sensor telemetry:
      telemetry.addData("LED", bLedOn ? "On" : "Off");
      telemetry.addData("Clear", colorSensor.alpha());
      telemetry.addData("Red  ", colorSensor.red());
      telemetry.addData("Green", colorSensor.green());
      telemetry.addData("Blue ", colorSensor.blue());
      telemetry.addData("Hue", hsvValues[0]);
      telemetry.addData("Default Hue", DEFAULTHUE);
      telemetry.addData("Distance Pixel Count:", distanceCountpixels);
      telemetry.addData("Color Pixel Count:", colorCountpixels);

      // Distance Sensor Telemetry:
      movingAverage1.add((float)(distanceSensor1.getDistance(DistanceUnit.CM)));
      telemetry.addData("range1", String.format("%.01f cm", distanceSensor1.getDistance(DistanceUnit.CM)));
      telemetry.addData("movingAverage1: ", movingAverage1.getAverage());
      telemetry.addData("DistanceStateMachine:", distanceSensorStateMachine);
      telemetry.addData("ColorStateMachine:", colorSensorState);

      // ADD TELEMETRY FOR PIXEL COUNT - Color
      // ADD TELEMETRY FOR PIXEL COUNT - Distance

      // change the background color to match the color detected by the RGB sensor.
      // pass a reference to the hue, saturation, and value array as an argument
      // to the HSVToColor method.
      relativeLayout.post(new Runnable() {
        public void run() {
          relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
        }
      });

      telemetry.update();
    }

    // Set the panel back to the default color
    relativeLayout.post(new Runnable() {
      public void run() {
        relativeLayout.setBackgroundColor(Color.WHITE);
      }
    });
  }
}
