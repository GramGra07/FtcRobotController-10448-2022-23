package org.firstinspires.ftc.teamcode.externalHardware;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "maintenance", group = "Robot")
//@Disabled
public class maintenanceXternal extends LinearOpMode {
    HardwareConfig robot = new HardwareConfig(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        while (opModeIsActive()) {//while the op mode is active
            if (robot.isSolid) {
                sleep(robot.delay * 1000);
                robot.lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(robot.getColor()));
                robot.isSolid = false;
            }
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
            if (robot.armUp) {
                robot.armEncoder(1800, 1, 6, false);
                robot.green1.setState(true);
                robot.red1.setState(false);
            } else {
                robot.armEncoder(0, 0.8, 6, true);
                robot.green1.setState(false);
                robot.red1.setState(true);
            }
            if (robot.clawOpen) {
                robot.openClaw();
                robot.green2.setState(false);
                robot.red2.setState(true);
            } else {
                robot.closeClaw();
                robot.green2.setState(true);
                robot.red2.setState(false);
            }
            telemetry.addData("armUp", robot.armUp);
            telemetry.addData("clawOpen", robot.clawOpen);
            telemetry.addData("color", robot.color);
            telemetry.update();
        }
    }
}