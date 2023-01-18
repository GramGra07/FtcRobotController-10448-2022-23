package org.firstinspires.ftc.teamcode.externalHardware;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "blankAuto", group = "Robot")
@Disabled
public class blankAuto extends LinearOpMode {
    autoHardware robot = new autoHardware(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        if (opModeIsActive()) {//while the op mode is active

        }
    }
}