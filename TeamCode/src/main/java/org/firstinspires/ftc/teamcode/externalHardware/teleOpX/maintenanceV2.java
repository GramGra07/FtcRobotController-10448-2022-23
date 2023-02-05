package org.firstinspires.ftc.teamcode.externalHardware.teleOpX;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.externalHardware.HardwareConfig;

@TeleOp(name = "maintenance2", group = "Robot")
//@Disabled
public class maintenanceV2 extends LinearOpMode {
    HardwareConfig robot = new HardwareConfig(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {//while the op mode is active
            if (isStopRequested()) return;

            //lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(getColor()));
            if (robot.touchSensor.isPressed()) {
                robot.armUp = !robot.armUp;
                robot.greenRed();
                robot.runtime.reset();
            }
            if (robot.touchSensorClaw.isPressed()) {
                robot.clawOpen = !robot.clawOpen;
                robot.greenRed();
                robot.runtime.reset();
            }
            if (robot.touchSensorEject.isPressed()) {
                robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(robot.getColor()));
                robot.runtime.reset();
            }
            if (robot.touchSensorL.isPressed()) {
                robot.greenRed();
                robot.tapeOut = !robot.tapeOut;
                robot.runtime.reset();
            }

            if (robot.tapeOut) {
                robot.tapeEncoder((int) (robot.countsPerInchTape * 18), 1, 6, false);//go out
                robot.green3.setState(true);
                robot.red3.setState(false);
            } else {
                robot.tapeEncoder(0, 1, 6, true);// come in
                robot.green3.setState(false);
                robot.red3.setState(true);
            }
            if (robot.clawOpen) {
                robot.tmPose -= 2;
                robot.tmServo.setPosition(robot.setServo(robot.tmPose));
                robot.green2.setState(false);
                robot.red2.setState(true);
                robot.clawOpen = false;
            }
            telemetry.addData("armUp", robot.armUp);
            telemetry.addData("clawOpen", robot.clawOpen);
            telemetry.addData("color", robot.color);
            telemetry.addData("tapeOut", robot.tapeOut);
            telemetry.addData("tmPose", robot.tmPose);
            telemetry.update();
        }
    }
}