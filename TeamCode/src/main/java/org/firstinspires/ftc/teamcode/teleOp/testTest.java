package org.firstinspires.ftc.teamcode.teleOp;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "testTest", group = "Robot")//declaring the name and group of the opmode
//@Disabled//disabling the opmode
public class testTest extends LinearOpMode {//declaring the class
    //other variables
    public boolean slowModeIsOn = false;//declaring the slowModeIsOn variable
    public boolean reversed = false;//declaring the reversed variable
    //motors/servos
    public DcMotor deadWheel = null;//declaring the deadWheel motor
    //public DcMotor deadWheelL = null;//declaring the deadWheelL motor
    //public DcMotor deadWheelR = null;//declaring the deadWheelR motor
    public DistanceSensor rDistance;//declaring the rDistance sensor
    public DistanceSensor lDistance;//declaring the lDistance sensor
    public DistanceSensor fDistance;//declaring the fDistance sensor
    public DcMotor motorFrontLeft = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackRight = null;
    public DcMotor sparkLong = null;
    public Servo clawServo = null;


    @Override
    public void runOpMode() {//if opmode is started
        ElapsedTime runtime = new ElapsedTime();//declaring the runtime variable
        rDistance = hardwareMap.get(DistanceSensor.class, "rDistance");//getting the rDistance sensor
        lDistance = hardwareMap.get(DistanceSensor.class, "lDistance");//getting the lDistance sensor
        fDistance = hardwareMap.get(DistanceSensor.class, "fDistance");//getting the fDistance sensor
        // Make sure your ID's match your configuration
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");//getting the motorFrontLeft motor
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");//getting the motorBackLeft motor
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");//getting the motorFrontRight motor
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");//getting the motorBackRight motor
        deadWheel = hardwareMap.get(DcMotor.class, "deadWheel");//getting the deadWheel motor
        //deadWheelL = hardwareMap.get(DcMotor.class, "deadWheelL");//getting the deadWheelL motor
        //deadWheelR = hardwareMap.get(DcMotor.class, "deadWheelR");//getting the deadWheelR motor
        clawServo = hardwareMap.get(Servo.class, "clawServo");//getting the clawServo servo
        sparkLong = hardwareMap.get(DcMotor.class, "sparkLong");//getting the sparkLong motor

        sparkLong.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the sparkLong encoder
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the motorFrontLeft encoder
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the motorBackRight encoder
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the motorBackLeft encoder
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the motorFrontRight encoder
        deadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the deadWheel encoder
        //deadWheelL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the deadWheelL encoder
        //deadWheelR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the deadWheelR encoder

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);//setting the motorBackRight direction

        sparkLong.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting the sparkLong encoder to run using encoder
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting the motorFrontLeft encoder to run using encoder
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting the motorBackLeft encoder to run using encoder
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting the motorBackRight encoder to run using encoder
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting the motorFrontRight encoder to run using encoder
        deadWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting the deadWheel encoder to run using encoder
        //deadWheelL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting the deadWheelL encoder to run using encoder
        //deadWheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting the deadWheelR encoder to run using encoder

        motorBackRight.setZeroPowerBehavior(BRAKE);
        motorBackLeft.setZeroPowerBehavior(BRAKE);
        motorFrontRight.setZeroPowerBehavior(BRAKE);
        motorFrontLeft.setZeroPowerBehavior(BRAKE);
        sparkLong.setZeroPowerBehavior(BRAKE);
        runtime.reset();//resetting the runtime variable
        waitForStart();//waiting for the start button to be pressed

        //if (isStopRequested()) return;//if the stop button is pressed, stop the program

        while (opModeIsActive()) {//while the op mode is active
            //switches
            if (gamepad1.left_trigger > 0) {
                slowModeIsOn = false;
            }
            if (gamepad1.right_trigger > 0) {
                slowModeIsOn = true;
            }
            double slowPower;
            if (slowModeIsOn) {
                slowPower = 3;
            } else {
                slowPower = 1;
            }
            double yControl = -gamepad1.left_stick_y;
            double xControl = gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;
            double frontRightPower = (yControl - xControl + turn) / slowPower;
            double backRightPower = (yControl + xControl + turn) / slowPower;
            double frontLeftPower = (yControl + xControl - turn) / slowPower;
            double backLeftPower = (yControl - xControl - turn) / slowPower;
            //
            //
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);
            telemetry.update();
        }
    }

    public String getColor() {
        final String[] favColors = {
                "RAINBOW_RAINBOW_PALETTE",
                "RAINBOW_PARTY_PALETTE",
                "BEATS_PER_MINUTE_RAINBOW_PALETTE",
                "BEATS_PER_MINUTE_PARTY_PALETTE",
                //"FIRE_MEDIUM",
                "COLOR_WAVES_RAINBOW_PALETTE",
                "COLOR_WAVES_PARTY_PALETTE",
                "CP2_END_TO_END_BLEND_TO_BLACK",
                "CP2_BREATH_SLOW",
                "CP1_2_END_TO_END_BLEND_1_TO_2",
                "CP1_2_END_TO_END_BLEND",
                "HOT_PINK",
                "GOLD",
                "VIOLET"
        };
        final int min = 0;
        final int max = favColors.length - 1;
        return favColors[(int) Math.floor(Math.random() * (max - min + 1) + min)];
    }

    public void setUniPower(double power) {
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackRight.setPower(power);
    }
    //public void unConeUp() {
    //    unConer.setPosition(setServo(magicUnCone));
    //    unConed = true;
    //}
    //public void unConeDown() {
    //    unConer.setPosition(setServo(baseUnCone));
    //    unConed = false;
    //}
    //public void ejectUp() {
    //    unConer.setPosition(setServo(magicEject));
    //}
    //public void ejectDown() {
    //    unConer.setPosition(setServo(baseEject));
    //}

    public void teleSpace() {
        telemetry.addLine();
    }

}