package org.firstinspires.ftc.teamcode.externalTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "external hardware test", group = "Robot")
//declaring the name and group of the opmode
//@Disabled//disabling the opmode
public class externalHardwareTest extends LinearOpMode {//declaring the class
    HardwareConfig robot = new HardwareConfig(this);

    @Override
    public void runOpMode() {//if opmode is started
        robot.updateStatus("Initializing");
        robot.init(hardwareMap);
        while (opModeIsActive()) {//while the op mode is active
            double armPower = -gamepad2.left_stick_y;
            if (gamepad1.dpad_up) {
                robot.assisting = !robot.assisting;
            }
            while (robot.assisting) {
                robot.assist();
            }
            if (gamepad2.dpad_down) {
                robot.sparkLong.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.sparkLong.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (gamepad1.touchpad_finger_1) {       //if the touchpad is pressed
                telemetry.addData("Finger 1: ", "%2f : %2f", gamepad1.touchpad_finger_1_x, gamepad1.touchpad_finger_1_y);
                if (gamepad1.touchpad_finger_1_x > 0) {//right side
                    robot.right = true;
                }
                if (gamepad1.touchpad_finger_1_x < 0) {//left side
                    robot.right = false;
                }
            }
            //
            if (robot.rumble) {
                if ((robot.runtime.seconds() > robot.endgame) && !robot.isEndgame) {
                    gamepad1.runRumbleEffect(robot.customRumbleEffect);
                    gamepad2.runRumbleEffect(robot.customRumbleEffect);
                    robot.isEndgame = true;
                }
                if ((robot.runtime.seconds() > robot.end) && !robot.isEnd) {
                    gamepad1.runRumbleEffect(robot.customRumbleEffect1);
                    gamepad2.runRumbleEffect(robot.customRumbleEffect1);
                    robot.isEnd = true;
                }
            }


            //switches
            if (gamepad1.left_trigger > 0) {
                robot.slowModeIsOn = false;
            }
            if (gamepad1.right_trigger > 0) {
                robot.slowModeIsOn = true;
            }
            if (robot.slowModeIsOn) {
                robot.slowPower = robot.slowMult;
            } else {
                robot.slowPower = 1;
            }
            //
            robot.yControl = -gamepad1.left_stick_y;
            robot.xControl = gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;
            double frontRightPower = (robot.yControl - robot.xControl + turn) / robot.slowPower;
            double backRightPower = (robot.yControl + robot.xControl + turn) / robot.slowPower;
            double frontLeftPower = (robot.yControl + robot.xControl - turn) / robot.slowPower;
            double backLeftPower = (robot.yControl - robot.xControl - turn) / robot.slowPower;
            //
            //arm extend controller 2
            robot.sparkLong.setPower(armPower);
            //
            //claw code
            if (gamepad2.left_bumper) {
                robot.clawServo.setPosition(robot.setServo(robot.magicNumOpen));
                robot.clawOpen = true;
                //open claw
            } else if (gamepad2.right_bumper) {
                robot.clawServo.setPosition(robot.setServo(robot.baseClawVal));
                //close claw
                robot.clawOpen = false;
            }//else{
            //    clawServo.setPosition(setServo(0));
            //}//auto close
            if (robot.clawOpen) {
                robot.green1.setState(false);
                robot.red1.setState(true);
            } else {
                robot.green1.setState(true);
                robot.red1.setState(false);
            }
            //
            //
            robot.motorFrontLeft.setPower(frontLeftPower);
            robot.motorBackLeft.setPower(backLeftPower);
            robot.motorFrontRight.setPower(frontRightPower);
            robot.motorBackRight.setPower(backRightPower);
            telemetry.addData("Status", robot.statusVal);//shows current status
            //telemetry.addLine("Arm")
            //        .addData("Val", String.valueOf(sparkLong.getCurrentPosition()))
            //        .addData("Max", armLimit)
            //        .addData("Limiter", limiter)
            //        .addData("Is broken", (sparkLong.getCurrentPosition() > armLimit));
            //.addData("Is Limiting",limiting);
            telemetry.addData("reversed", robot.reversed);
            telemetry.addData("slowMode", robot.slowModeIsOn);
            telemetry.addData("dead", robot.deadWheel.getCurrentPosition());
            //telemetry.addData("deadR", deadWheelR.getCurrentPosition());
            //telemetry.addData("deadL", deadWheelL.getCurrentPosition());
            robot.teleSpace();
            //getAllColor();
            //teleSpace();
            //distanceTelemetry();
            robot.updateStatus("Running");
            telemetry.update();
        }
    }

}