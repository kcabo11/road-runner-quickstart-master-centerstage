/*
Copyright (c) 2018 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.MovingAverage;

/*
 * This OpMode illustrates how to use the REV Robotics 2M Distance Sensor.
 *
 * The OpMode assumes that the sensor is configured with a name of "sensor_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * See the sensor's product page: https://www.revrobotics.com/rev-31-1505/
 */
@TeleOp(name = "Sensor: REV2mDistance", group = "Sensor")
//@Disabled
public class SensorREV2mDistance extends LinearOpMode {

    private DistanceSensor sensorDistance;
    private MovingAverage movingAverage1 = new MovingAverage(10);
    private MovingAverage movingAverage2 = new MovingAverage(10);

    public DistanceSensor distanceSensor1;
    public DistanceSensor distanceSensor2;

    @Override
    public void runOpMode() {

        // you can use this as a regular DistanceSensor.
        distanceSensor1 = hardwareMap.get(DistanceSensor.class, "distanceSensor1");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "distanceSensor2");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight1 = (Rev2mDistanceSensor) distanceSensor1;
        Rev2mDistanceSensor sensorTimeOfFlight2 = (Rev2mDistanceSensor) distanceSensor2;

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            // Distance Sensor 1 Telemetry
            // generic DistanceSensor methods.
            movingAverage1.add((float)(distanceSensor1.getDistance(DistanceUnit.CM)));
//            telemetry.addData("distanceSensor1", distanceSensor1.getDeviceName() );
//            telemetry.addData("range", String.format("%.01f mm", distanceSensor1.getDistance(DistanceUnit.MM)));
            telemetry.addData("range1", String.format("%.01f cm", distanceSensor1.getDistance(DistanceUnit.CM)));
//            telemetry.addData("range", String.format("%.01f m", distanceSensor1.getDistance(DistanceUnit.METER)));
//            telemetry.addData("range", String.format("%.01f in", distanceSensor1.getDistance(DistanceUnit.INCH)));
            telemetry.addData("movingAverage1: ", movingAverage1.getAverage());


            // Distance Sensor 2 Telemetry
            // generic DistanceSensor methods.
            movingAverage2.add((float)(distanceSensor2.getDistance(DistanceUnit.CM)));
//            telemetry.addData("distanceSensor2", distanceSensor2.getDeviceName() );
//            telemetry.addData("range", String.format("%.01f mm", distanceSensor2.getDistance(DistanceUnit.MM)));
            telemetry.addData("range2", String.format("%.01f cm", distanceSensor2.getDistance(DistanceUnit.CM)));
//            telemetry.addData("range", String.format("%.01f m", distanceSensor2.getDistance(DistanceUnit.METER)));
//            telemetry.addData("range", String.format("%.01f in", distanceSensor2.getDistance(DistanceUnit.INCH)));
            telemetry.addData("movingAverage2: ", movingAverage2.getAverage());

            // Rev2mDistanceSensor specific methods.
            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight2.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight2.didTimeoutOccur()));

            telemetry.update();
        }
    }

}
