/*
 * Copyright (c) 2018 Craig MacFarlane
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * (subject to the limitations in the disclaimer below) provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Craig MacFarlane nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. THIS
 * SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Display patterns of a REV Robotics Blinkin LED Driver.
 * AUTO mode cycles through all of the patterns.
 * MANUAL mode allows the user to manually change patterns using the
 * left and right bumpers of a gamepad.
 *
 * Configure the driver on a servo port, and name it "blinkin".
 *
 * Displays the first pattern upon init.
 */
@TeleOp(name="BlinkinTest")
//@Disabled
public class BlinkinTest extends OpMode {

    RevBlinkinLedDriver lights;
    private static final String[] favColors = {
            "GREEN",
            "RED",
            "BLUE"
    };
    public int colorIndex = 0;

    @Override
    public void init()
    {
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(favColors[colorIndex]));
    }

    @Override
    public void loop()
    {
        if (gamepad1.right_bumper){
            colorIndex++;
            if (colorIndex >= favColors.length){
                colorIndex = 0;
            }
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(favColors[colorIndex]));
        }
        if (gamepad1.left_bumper){
            colorIndex--;
            if (colorIndex < 0){
                colorIndex = favColors.length - 1;
            }
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(favColors[colorIndex]));
        }
    }
}
