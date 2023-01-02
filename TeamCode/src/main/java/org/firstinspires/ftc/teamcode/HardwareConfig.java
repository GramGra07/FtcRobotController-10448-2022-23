//import
package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
public class HardwareConfig {
    //motors
    public DcMotor motorFrontLeft = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackRight = null;
    public DcMotor deadWheel = null;//declaring the deadWheel motor
    //servo
    public DcMotor sparkLong = null;
    public TouchSensor touchSensor;
    public NormalizedColorSensor colorSensorR;//declaring the colorSensor variable
    public NormalizedColorSensor colorSensorL;//declaring the colorSensor variable
    public DigitalChannel red1;
    public DigitalChannel green1;
    public DigitalChannel red2;
    public DigitalChannel green2;
    public DigitalChannel red3;
    public DigitalChannel green3;
    public DigitalChannel red4;
    public DigitalChannel green4;
    public DistanceSensor rDistance;//declaring the rDistance sensor
    public DistanceSensor lDistance;//declaring the lDistance sensor
    public DistanceSensor fDistance;//declaring the fDistance sensor
    public RevBlinkinLedDriver lights;
    public Servo clawServo = null;
    public BNO055IMU imu;
    HardwareMap hardwareMap = null;

    public void init(HardwareMap ahwMap) {
        ElapsedTime runtime = new ElapsedTime();//declaring the runtime variable
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        rDistance = hardwareMap.get(DistanceSensor.class, "rDistance");//getting the rDistance sensor
        lDistance = hardwareMap.get(DistanceSensor.class, "lDistance");//getting the lDistance sensor
        fDistance = hardwareMap.get(DistanceSensor.class, "fDistance");//getting the fDistance sensor
        red1 = hardwareMap.get(DigitalChannel.class, "red1");//getting the red1 light
        green1 = hardwareMap.get(DigitalChannel.class, "green1");//getting the green1 light
        red2 = hardwareMap.get(DigitalChannel.class, "red2");//getting the red2 light
        green2 = hardwareMap.get(DigitalChannel.class, "green2");//getting the green2 light
        red3 = hardwareMap.get(DigitalChannel.class, "red3");//getting the red3 light
        green3 = hardwareMap.get(DigitalChannel.class, "green3");//getting the green3 light
        red4 = hardwareMap.get(DigitalChannel.class, "red4");//getting the red4 light
        green4 = hardwareMap.get(DigitalChannel.class, "green4");//getting the green4 light
        colorSensorR = hardwareMap.get(NormalizedColorSensor.class, "colorSensorR");
        colorSensorL = hardwareMap.get(NormalizedColorSensor.class, "colorSensorL");
        // Declare our motors
        // Make sure your ID's match your configuration
        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");//getting the motorFrontLeft motor
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");//getting the motorBackLeft motor
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");//getting the motorFrontRight motor
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");//getting the motorBackRight motor
        deadWheel = hardwareMap.get(DcMotor.class, "deadWheel");//getting the deadWheel motor
        clawServo = hardwareMap.get(Servo.class, "clawServo");//getting the clawServo servo
        sparkLong = hardwareMap.get(DcMotor.class, "sparkLong");//getting the sparkLong motor
        touchSensor = hardwareMap.get(TouchSensor.class, ("touchSensor"));

        sparkLong.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the sparkLong encoder
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the motorFrontLeft encoder
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the motorBackRight encoder
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the motorBackLeft encoder
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the motorFrontRight encoder
        deadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the deadWheel encoder
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);//setting the motorFrontRight direction
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);//setting the motorBackRight direction
        sparkLong.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting the sparkLong encoder to run using encoder
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting the motorFrontLeft encoder to run using encoder
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting the motorBackLeft encoder to run using encoder
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting the motorBackRight encoder to run using encoder
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting the motorFrontRight encoder to run using encoder
        deadWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//setting the deadWheel encoder to run using encoder

        motorBackRight.setZeroPowerBehavior(BRAKE);
        motorBackLeft.setZeroPowerBehavior(BRAKE);
        motorFrontRight.setZeroPowerBehavior(BRAKE);
        motorFrontLeft.setZeroPowerBehavior(BRAKE);
        sparkLong.setZeroPowerBehavior(BRAKE);
        red1.setMode(DigitalChannel.Mode.OUTPUT);//setting the red1 light to output
        green1.setMode(DigitalChannel.Mode.OUTPUT);//setting the green1 light to output
        red2.setMode(DigitalChannel.Mode.OUTPUT);//setting the red2 light to output
        green2.setMode(DigitalChannel.Mode.OUTPUT);//setting the green2 light to output
        red3.setMode(DigitalChannel.Mode.OUTPUT);//setting the red3 light to output
        green3.setMode(DigitalChannel.Mode.OUTPUT);//setting the green3 light to output
        red4.setMode(DigitalChannel.Mode.OUTPUT);//setting the red4 light to output
        green4.setMode(DigitalChannel.Mode.OUTPUT);//setting the green4 light to output

        //flipper.setPosition(setServo(magicFlip));//setting the flipper servo to the magicFlip position
        runtime.reset();//resetting the runtime variable
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }
}
