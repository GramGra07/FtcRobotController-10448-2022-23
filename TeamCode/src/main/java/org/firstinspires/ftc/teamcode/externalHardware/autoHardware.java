package org.firstinspires.ftc.teamcode.externalHardware;

import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class autoHardware extends HardwareConfig {
    double ovrPower = 0.5;
    HardwareMap hardwareMap = null;

    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    public autoHardware(LinearOpMode opMode) {
        super(opMode);
        myOpMode = opMode;
    }

    public void initAuto(HardwareMap ahwMap) {
        hardwareMap = ahwMap;
        updateStatus("Initializing");
        ElapsedTime runtime = new ElapsedTime();//declaring the runtime variable
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = ahwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        lights = ahwMap.get(RevBlinkinLedDriver.class, "blinkin");
        rDistance = ahwMap.get(DistanceSensor.class, "rDistance");//getting the rDistance sensor
        lDistance = ahwMap.get(DistanceSensor.class, "lDistance");//getting the lDistance sensor
        fDistance = ahwMap.get(DistanceSensor.class, "fDistance");//getting the fDistance sensor
        red1 = ahwMap.get(DigitalChannel.class, "red1");//getting the red1 light
        green1 = ahwMap.get(DigitalChannel.class, "green1");//getting the green1 light
        red2 = ahwMap.get(DigitalChannel.class, "red2");//getting the red2 light
        green2 = ahwMap.get(DigitalChannel.class, "green2");//getting the green2 light
        red3 = ahwMap.get(DigitalChannel.class, "red3");//getting the red3 light
        green3 = ahwMap.get(DigitalChannel.class, "green3");//getting the green3 light
        red4 = ahwMap.get(DigitalChannel.class, "red4");//getting the red4 light
        green4 = ahwMap.get(DigitalChannel.class, "green4");//getting the green4 light
        colorSensorR = ahwMap.get(NormalizedColorSensor.class, "colorSensorR");
        colorSensorL = ahwMap.get(NormalizedColorSensor.class, "colorSensorL");
        // Declare our motors
        // Make sure your ID's match your configuration
        motorFrontLeft = ahwMap.get(DcMotor.class, "motorFrontLeft");//getting the motorFrontLeft motor
        motorBackLeft = ahwMap.get(DcMotor.class, "motorBackLeft");//getting the motorBackLeft motor
        motorFrontRight = ahwMap.get(DcMotor.class, "motorFrontRight");//getting the motorFrontRight motor
        motorBackRight = ahwMap.get(DcMotor.class, "motorBackRight");//getting the motorBackRight motor
        deadWheel = ahwMap.get(DcMotor.class, "deadWheel");//getting the deadWheel motor
        clawServo = ahwMap.get(Servo.class, "clawServo");//getting the clawServo servo
        sparkLong = ahwMap.get(DcMotor.class, "sparkLong");//getting the sparkLong motor
        touchSensor = ahwMap.get(TouchSensor.class, ("touchSensor"));
        touchSensorL = ahwMap.get(TouchSensor.class, ("touchSensorL"));
        touchSensorClaw = ahwMap.get(TouchSensor.class, ("touchSensorClaw"));
        touchSensorEject = ahwMap.get(TouchSensor.class, ("touchSensorEject"));
        tapeMeasure = ahwMap.get(DcMotor.class, "tapeMeasure");

        sparkLong.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the sparkLong encoder
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the motorFrontLeft encoder
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the motorBackRight encoder
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the motorBackLeft encoder
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the motorFrontRight encoder
        deadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//resetting the deadWheel encoder

        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        tapeMeasure.setDirection(DcMotor.Direction.REVERSE);
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
        if (myOpMode.isStopRequested()) return;
        myOpMode.telemetry.addData("Starting at", "%7d :%7d",
                motorBackRight.getCurrentPosition(),
                motorBackLeft.getCurrentPosition(),
                motorFrontLeft.getCurrentPosition());
        closeClaw();
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0 / 9.0);
        }
        runVu(6, false);
        myOpMode.telemetry.update();
        closeClaw();
        // Wait for the game to start (driver presses PLAY)
        sleep(300);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        myOpMode.waitForStart();
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(getColor()));
    }

    public void correctByImu(float currentAngle, int targetAngle) {
        int angle = (int) (targetAngle - currentAngle);
        turn(angle);
    }

    public void correctToCones() {
        correctByColor();
        correctByTouch();
    }

    public void correctByColor() {
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        getAllColorR();
        getAllColorL();
        NormalizedRGBA colorsR = colorSensorR.getNormalizedColors();
        Color.colorToHSV(colorsR.toColor(), hsvValues);
        NormalizedRGBA colors = colorSensorL.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        float redValR = colorsR.red;//the red value in rgb
        float greenValR = colorsR.green;//the green value in rgb
        float blueValR = colorsR.blue;//the blue value in rgb
        float redValL = colors.red;//the red value in rgb
        float greenValL = colors.green;//the green value in rgb
        float blueValL = colors.blue;//the blue value in rgb
        //right
        double redTargetRR = 0.003;//the red value in rgb
        double redTargetGR = 0.004;//the green value in rgb
        double redTargetBR = 0.003;//the blue value in rgb
        //left
        double redTargetRL = 0.003;//the red value in rgb
        double redTargetGL = 0.003;//the green value in rgb
        double redTargetBL = 0.002;//the blue value in rgb
        //right
        double blueTargetRR = 0.002;//the red value in rgb
        double blueTargetGR = 0.004;//the green value in rgb
        double blueTargetBR = 0.005;//the blue value in rgb
        //left
        double blueTargetRL = 0.001;//the red value in rgb
        double blueTargetGL = 0.003;//the green value in rgb
        double blueTargetBL = 0.0038;//the blue value in rgb
        double range = 0.0005;
        //left
        while (colorInRange(redValL, redTargetRL, greenValL, redTargetGL, blueValL, redTargetBL, (float) range)
                || colorInRange(redValL, blueTargetRL, greenValL, blueTargetGL, blueValL, blueTargetBL, (float) range)
                || colorInRange(redValR, redTargetRR, greenValR, redTargetGR, blueValR, redTargetBR, (float) range)
                || colorInRange(redValR, blueTargetRR, greenValR, blueTargetGR, blueValR, blueTargetBR, (float) range)) {
            if ((colorInRange(redValR, redTargetRR, greenValR, redTargetGR, blueValR, redTargetBR, (float) range)
                    || colorInRange(redValR, blueTargetRR, greenValR, blueTargetGR, blueValR, blueTargetBR, (float) range))) {
                getAllColorR();
                sideWaysEncoderDrive(1, 0.25, 0.4);//go left
                //right side has seen red or blue
            }
            if (colorInRange(redValL, redTargetRL, greenValL, redTargetGL, blueValL, redTargetBL, (float) range)
                    || colorInRange(redValL, blueTargetRL, greenValL, blueTargetGL, blueValL, blueTargetBL, (float) range)) {
                getAllColorL();
                sideWaysEncoderDrive(1, -0.25, 0.4);//go right
            }
            if (!colorInRange(redValL, redTargetRL, greenValL, redTargetGL, blueValL, redTargetBL, (float) range)
                    || !colorInRange(redValL, blueTargetRL, greenValL, blueTargetGL, blueValL, blueTargetBL, (float) range)
                    || !colorInRange(redValR, redTargetRR, greenValR, redTargetGR, blueValR, redTargetBR, (float) range)
                    || !colorInRange(redValR, blueTargetRR, greenValR, blueTargetGR, blueValR, blueTargetBR, (float) range)) {
                break;
            }
        }
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(getColor()));
    }


    public void correctByTouch() {
        boolean pressed = touchSensor.isPressed();
        boolean pressedL = touchSensorL.isPressed();
        while (!pressed && !pressedL) {
            pressed = touchSensor.isPressed();
            pressedL = touchSensorL.isPressed();
            if (pressed && pressedL) {
                break;
            }
            encoderDrive(1, -6, -6, 1);
        }
    }

    public void doSetup() {
        runVu(6, true);
        if (spot == 0) {
            spot = 2;
        }
        reportSpot(spot);
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(getColor()));
    }

    public void reportSpot(int spot) {
        if (spot == 1) {
            green1.setState(true);
            red1.setState(false);
        } else if (spot == 2) {
            green1.setState(true);
            red1.setState(false);
            green2.setState(true);
            red2.setState(false);
        } else if (spot == 3) {
            green1.setState(true);
            red1.setState(false);
            green2.setState(true);
            red2.setState(false);
            green3.setState(true);
            red3.setState(false);
        }
    }

    public void score1() {
        doSetup();
        //branch 1 get to spot
        encoderDrive(1, -1, -1, 0.5);
        simplerGoSpot(ovrCurrX, ovrCurrY, 1, 2, ovrPower, false, 0, false
                , false, 0, 1, 4);
        setOvr(1, 2);
        double targetX = -0.02;
        double targetY2 = 3.55;// at pole
        sleep(50);
        resetEncoders();
        simplerGoSpot(1, 3, targetX, targetY2, ovrPower, true, topPoleVal,
                false, false, 0, 2, 4);
        turn(80);
        resetEncoders();
        double fw = -2;
        encoderDrive(1, fw, fw, 1);
        setOvr(targetX, targetY2);
        sleep(500);
        openClaw();
        sleep(200);
        closeClaw();
        sleep(200);
        encoderDrive(1, -fw, -fw, 1);
    }

    public void simpleGoSpotRight(double currX, double currY, double targetX, double targetY, double power,
                                  boolean combo, int pose, boolean isUp, boolean endTurn, int turn, double timeOutX,
                                  double timeOutY, boolean prioritizeY) {
        double XMULT = 9.0;
        double YMULT = 20.0;
        double sidewaysInches = (targetY - currY) * XMULT * -1;
        double fwdInches = (targetX - currX) * YMULT;
        if (prioritizeY) {
            sideWaysEncoderDrive(power, sidewaysInches, timeOutY);
            sleep(100);
        }
        if (!combo) {
            encoderDrive(power, fwdInches, fwdInches, timeOutX);
        } else {
            encoderComboFwd(power, fwdInches, fwdInches, pose, timeOutX, isUp);
        }
        if (!prioritizeY) {
            sleep(100);
            sideWaysEncoderDrive(power, sidewaysInches, timeOutY);
        }
        if (endTurn) {
            turn(turn);
        }
        setOvr(targetX, targetY);
        myOpMode.telemetry.update();
    }

    public void simplerGoSpot(double currX, double currY, double targetX, double targetY, double power, boolean combo, int pose
            , boolean isUp, boolean endTurn, int turn, int timeOutX, int timeOutY) {
        double sidewaysInches = (targetX - currX) * xMult;
        double fwdInches = (targetY - currY) * yMult;
        myOpMode.telemetry.addData("fwdInches", fwdInches);
        myOpMode.telemetry.addData("sidewaysInches", sidewaysInches);
        myOpMode.telemetry.update();
        sideWaysEncoderDrive(power, sidewaysInches, timeOutX);
        if (!combo) {
            encoderDrive(power, -fwdInches, -fwdInches, timeOutY);
        } else {
            encoderComboFwd(power, -fwdInches, -fwdInches, pose, timeOutY, isUp);
        }
        if (endTurn) {
            turn(turn);
        }
        setOvr(targetX, targetY);
        myOpMode.telemetry.update();
    }
}
