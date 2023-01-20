package org.firstinspires.ftc.teamcode.externalHardware;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "simpAutoRX", group = "Robot")
//@Disabled
public class simpAutoRX extends LinearOpMode {
    autoHardware robot = new autoHardware(this);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.initAuto(hardwareMap);
        if (opModeIsActive()) {//while the op mode is active
            robot.score1();
            double stackDist = -15;
            robot.sideWaysEncoderDrive(robot.ovrPower, -6, 2);
            robot.armEncoder(0, 1, 2, true);
            sleep(50);
            if (robot.spot == 3) {
                robot.encoderDrive(1, stackDist, stackDist, 3);//opposite of 3 lines higher
                //3,3
            }
            //should already be here at spot 2
            if (robot.spot == 2) {
                //2,3
            }
            if (robot.spot == 1) {
                robot.encoderDrive(1, -stackDist, -stackDist, 3);
                //1,3
            }
            telemetry.update();
        }
    }
}