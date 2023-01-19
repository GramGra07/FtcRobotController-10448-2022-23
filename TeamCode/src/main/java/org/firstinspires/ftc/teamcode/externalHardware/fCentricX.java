package org.firstinspires.ftc.teamcode.externalHardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "fCentricX", group = "Robot")
//@Disabled//disabling the opmode
public class fCentricX extends LinearOpMode {//declaring the class
    HardwareConfig robot = new HardwareConfig(this);

    @Override
    public void runOpMode() {//if opmode is started
        robot.init(hardwareMap);
        while (opModeIsActive()) {//while the op mode is active
            robot.doBulk(true);
        }
    }

}