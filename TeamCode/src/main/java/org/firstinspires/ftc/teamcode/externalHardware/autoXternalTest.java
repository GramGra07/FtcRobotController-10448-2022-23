package org.firstinspires.ftc.teamcode.externalHardware;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "autoXternalTest", group = "Robot")
//@Disabled
public class autoXternalTest extends LinearOpMode {
    autoHardwareConfig robot = new autoHardwareConfig(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        if (opModeIsActive()) {//while the op mode is active
            robot.encoderDrive(1, 10, 10, 10);
        }
    }
}