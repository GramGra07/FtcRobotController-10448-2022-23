package org.firstinspires.ftc.teamcode.auto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.teleOp.scrap;

import java.util.List;
import java.util.Objects;

@Autonomous(name = "autoLSimp", group = "Robot")
//@Disabled
public class autoLSimp extends LinearOpMode {

    private DcMotor motorFrontLeft = null;//fl
    private DcMotor motorBackLeft = null;//bl
    private DcMotor motorFrontRight = null;//fr
    private DcMotor motorBackRight = null;//br
    private DcMotor deadWheel = null;//dead strafe wheel
    private DcMotor deadWheelL = null;//left side dead wheel
    private DcMotor deadWheelR = null;//right side dead wheel
    private DcMotor sparkLong = null;//arm motor
    private Servo clawServo = null;//claw close servo

    private ElapsedTime runtime = new ElapsedTime();//to figure out time

    static final double COUNTS_PER_MOTOR_REV = 28;//given by rev
    static final double WHEEL_DIAMETER_MM = 96;//found online/measured
    static final double WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_MM * 0.0393701;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * 15) /
            (WHEEL_DIAMETER_INCHES * Math.PI);//gets the overall counts per inch to help with encoders

    static final double COUNTS_PER_MOTOR_REV_dead = 8192;//given by rev
    static final double WHEEL_DIAMETER_MM_dead = 96;
    static final double WHEEL_DIAMETER_INCHES_dead = WHEEL_DIAMETER_MM_dead * 0.0393701;     // For figuring circumference
    static final double COUNTS_PER_INCH_dead = (COUNTS_PER_MOTOR_REV_dead) /
            (WHEEL_DIAMETER_INCHES_dead * Math.PI);//gets the overall counts per inch to help with encoders


    //arm
    final int baseArmPosition = 0;
    public final int armLimit = scrap.armLimit;
    public final int lowPoleVal = scrap.lowPoleVal;//should be about 1/3 of arm limit
    public final int midPoleVal = scrap.midPoleVal;//should be about 2/3 of arm limit
    public final int topPoleVal = armLimit;//should be close to armLimit
    static final double COUNTS_PER_MOTOR_REV_arm = 28;
    static final double DRIVE_GEAR_REDUCTION_arm = 40;
    static final double WHEEL_DIAMETER_INCHES_arm = 1.102;     // For figuring circumference
    static final double COUNTS_PER_INCH_arm = (COUNTS_PER_MOTOR_REV_arm * DRIVE_GEAR_REDUCTION_arm) /
            (WHEEL_DIAMETER_INCHES_arm * 3.1415);

    static final double COUNTS_PER_INCH_Side_dead = -665.08;
    static final double COUNTS_PER_INCH_Side = -100;
    public final int baseClawVal = 30;
    public final int magicNumOpen = 60;
    public double position = 0;//sets servo position to 0-1 multiplier
    public final double degree_mult = 0.00555555554;//100/180

    private static final String TFOD_MODEL_ASSET = "custom.tflite";

    private static final String[] LABELS = {
            "capacitor",//3
            "led",//1
            "resistor"//2
    };

    private static final String VUFORIA_KEY =
            "AXmzBcj/////AAABme5HSJ/H3Ucup73WSIaV87tx/sFHYaWfor9OZVg6afr2Bw7kNolHd+mF5Ps91SlQpgBHulieI0jcd86kqJSwx46BZ8v8DS5S5x//eQWMEGjMDnvco4/oTcDwuSOLIVZG2UtLmJXPS1L3CipjabePFlqAL2JtBlN78p6ZZbRFSHW680hWEMSimZuQy/cMudD7J/MjMjMs7b925b8BkijlnTQYr7CbSlXrpDh5K+9fLlk2OyEZ4w7tm7e4UJDInJ/T3oi8PqqKCqkUaTkJWlQsvoELbDu5L2FgzsuDhBLe2rHtJRqfORd7n+6M30UdFSsxqq5TaZztkWgzRUr1GC3yBSTS6iFqEuL3g06GrfwOJF0F";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;
    public int spot = 0;

    private DistanceSensor rDistance;
    private DistanceSensor lDistance;
    private DistanceSensor fDistance;
    private DistanceSensor bDistance;
    public double IN_distanceR = 0;//in distance for distance sensor 1
    public double IN_distanceL = 0;
    public double myMagic = 7;

    DigitalChannel red2;
    DigitalChannel green2;
    public int turn = 76;
    public RevBlinkinLedDriver lights;

    @Override
    public void runOpMode() {
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        rDistance = hardwareMap.get(DistanceSensor.class, "rDistance");
        lDistance = hardwareMap.get(DistanceSensor.class, "lDistance");
        fDistance = hardwareMap.get(DistanceSensor.class, "fDistance");
        bDistance = hardwareMap.get(DistanceSensor.class, "bDistance");
        DigitalChannel red1 = hardwareMap.get(DigitalChannel.class, "red1");
        DigitalChannel green1 = hardwareMap.get(DigitalChannel.class, "green1");
        red2 = hardwareMap.get(DigitalChannel.class, "red2");
        green2 = hardwareMap.get(DigitalChannel.class, "green2");

        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        deadWheel = hardwareMap.get(DcMotor.class, "deadWheel");
        deadWheelL = hardwareMap.get(DcMotor.class, "deadWheelL");
        deadWheelR = hardwareMap.get(DcMotor.class, "deadWheelR");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        sparkLong = hardwareMap.get(DcMotor.class, "sparkLong");

        //onInit();
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders();
        sparkLong.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sparkLong.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        deadWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        deadWheelL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        deadWheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorBackRight.setZeroPowerBehavior(BRAKE);
        motorBackLeft.setZeroPowerBehavior(BRAKE);
        motorFrontRight.setZeroPowerBehavior(BRAKE);
        motorFrontLeft.setZeroPowerBehavior(BRAKE);

        closeClaw();
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0 / 9.0);
        }
        runVu(6, false);
        telemetry.addData("spot", spot);
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        waitForStart();
        if (opModeIsActive()) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(getColor()));
            runVu(6, true);
            if (spot == 0) {
                spot += 4;
            }
            flash(spot);
            closeClaw();
            sleep(500);
            armEncoder(1200, 1, 2, false);//get up a little higher
// constant steps
            encoderDrive(1, 6, 6, 1);
            turn(180);
            encoderDrive(1, 6, 6, 1);
            encoderDrive(0.5, -47, -47, 6);
//
            sideWaysEncoderDrive(0.5, -4.15, 1);
//
            armEncoder(topPoleVal, 1, 6, false);//go up
            encoderDrive(1, -4, -4, 1);
            sleep(500);
            armEncoder(topPoleVal - 100, 1, 2, true);
            openClaw();
            sleep(500);
            closeClaw();
            sideWaysEncoderDrive(1, 4, 1);
            armEncoder(baseArmPosition, 1, 6, true);//go back to base
//
            if (spot == 1) {
                //in-progress
                sideWaysEncoderDrive(0.8, 12, 4);
                situate();
            }
            if (spot == 2) {
                //in-progress
                sideWaysEncoderDrive(0.8, 2, 1);
                situate();
            }
            if (spot == 3) {
                sideWaysEncoderDrive(0.8, -8, 1);
                situate();
            }
            if (spot == 4) {
                sideWaysEncoderDrive(1, 4, 1);
                encoderDrive(1, 50, 50, 4);
                situate();
            }
//
        }

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    public String getColor(){
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
        final int min=0;
        final int max= favColors.length-1;
        return favColors[(int) Math.floor(Math.random() * (max - min + 1) + min)];
    }
    //precise if exact 180, if not, then use the following
    //final int actualF=50;
    //final int actualR=100;
    //final int actualL=44;
    //double myMagic2;
    //if ( fDistance.getDistance(DistanceUnit.INCH)<actualF){
    //    myMagic2=actualF-fDistance.getDistance(DistanceUnit.INCH);
    //    encoderDrive(0.75,-myMagic2,-myMagic2,3);
    //}
    //the following
    //while(fDistance.getDistance(DistanceUnit.INCH)<actualF){
    //    telemetry.addData("is working",fDistance.getDistance(DistanceUnit.INCH)<actualF);
    //    telemetry.addData("inches",fDistance.getDistance(DistanceUnit.INCH));
    //    telemetry.addData("actualF",actualF);
    //    telemetry.update();
    //    motorBackLeft.setPower(-0.8);
    //    motorBackRight.setPower(-0.8);
    //    motorFrontLeft.setPower(-0.8);
    //    motorFrontRight.setPower(-0.8);
    //}
    public void flash(int repetitions) {
        if (repetitions == 1) {
            red2.setState(false);
            green2.setState(true);
            sleep(50);
            red2.setState(true);
            green2.setState(false);
            sleep(50);
        }
        if (repetitions == 2) {
            red2.setState(false);
            green2.setState(true);
            sleep(50);
            red2.setState(true);
            green2.setState(false);
            sleep(50);
            red2.setState(false);
            green2.setState(true);
            sleep(50);
            red2.setState(true);
            green2.setState(false);
            sleep(50);
        }
        if (repetitions == 3) {
            red2.setState(false);
            green2.setState(true);
            sleep(50);
            red2.setState(true);
            green2.setState(false);
            sleep(50);
            red2.setState(false);
            green2.setState(true);
            sleep(50);
            red2.setState(true);
            green2.setState(false);
            sleep(50);
            red2.setState(false);
            green2.setState(true);
            sleep(50);
            red2.setState(true);
            green2.setState(false);
            sleep(50);
        }
        if (repetitions == 4) {
            red2.setState(true);
        }

    }

    public void turn(int degrees) {
        if (degrees > 180) {
            degrees = (360 - degrees) * -1;
        }
        int mult = 360 / degrees;
        int inches = (int) (turn / mult);
        encoderDrive(0.65, -inches, inches, 6);
        resetEncoders();
    }

    public void situate() {
        encoderDrive(1, 4, 4, 1);
    }

    public void resetEncoders() {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadWheelL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadWheelR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //sparkLong.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public double setServo(int degrees) {
        position = degree_mult * degrees;
        return position;
    }

    public void openClaw() {
        clawServo.setPosition(setServo(magicNumOpen));
    }

    public void closeClaw() {
        clawServo.setPosition(setServo(baseClawVal));
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newDLeftTarget;
        int newDRightTarget;
        if (opModeIsActive()) {

            newLeftTarget = motorBackLeft.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = motorBackRight.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newDLeftTarget = deadWheelL.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH_dead);
            newDRightTarget = deadWheelR.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH_dead);

            deadWheelL.setTargetPosition(-newDLeftTarget);
            deadWheelR.setTargetPosition(-newDRightTarget);
            motorFrontRight.setTargetPosition(-newRightTarget);
            motorBackRight.setTargetPosition(-newRightTarget);
            motorFrontLeft.setTargetPosition(-newLeftTarget);
            motorBackLeft.setTargetPosition(-newLeftTarget);

            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            deadWheelL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            deadWheelR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            motorBackLeft.setPower((speed));
            motorFrontRight.setPower((speed));
            motorFrontLeft.setPower((speed));
            motorBackRight.setPower((speed));
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorBackLeft.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", "%7d :%7d", -newDLeftTarget, -newDRightTarget);//"%7d :%7d"
                telemetry.addData("Currently at", "%7d :%7d",
                        deadWheelL.getCurrentPosition(), deadWheelR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorBackLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackRight.setPower(0);
            motorFrontLeft.setPower(0);

            // Turn off RUN_TO_POSITION
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            deadWheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            deadWheelR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            resetEncoders();
        }
    }

    public void sideWaysEncoderDrive(double speed,
                                     double inches,
                                     double timeoutS) {//+=right //-=left
        int newFRTarget;
        int newFLTarget;
        int newBRTarget;
        int newBLTarget;
        int newDeadTarget;
        inches *= -1;
        if (opModeIsActive()) {
            if (inches < 0) {
                newFLTarget = motorFrontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side);
                newBLTarget = motorBackLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side);
                newFRTarget = motorFrontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side);
                newBRTarget = motorBackRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side);
                newDeadTarget = deadWheel.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side_dead);
                motorFrontLeft.setTargetPosition(-newFLTarget);
                motorBackLeft.setTargetPosition(newBLTarget);
                motorBackRight.setTargetPosition(-newBRTarget - 10);
                motorFrontRight.setTargetPosition(newFRTarget);
                deadWheel.setTargetPosition(-newDeadTarget);
            }
            if (inches > 0) {
                newFLTarget = motorFrontLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side);
                newBLTarget = motorBackLeft.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side);
                newFRTarget = motorFrontRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side);
                newBRTarget = motorBackRight.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side);
                newDeadTarget = deadWheel.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side_dead);
                motorFrontLeft.setTargetPosition(-newFLTarget);
                motorBackLeft.setTargetPosition(newBLTarget);
                motorBackRight.setTargetPosition(-newBRTarget);
                motorFrontRight.setTargetPosition(newFRTarget);
                deadWheel.setTargetPosition(newDeadTarget);
            }

            motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            deadWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            motorBackLeft.setPower(Math.abs(speed));
            motorFrontRight.setPower(Math.abs(speed));
            motorFrontLeft.setPower(Math.abs(speed));
            motorBackRight.setPower(Math.abs(speed));
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) && deadWheel.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Running to", "%7d:%7d", motorFrontLeft.getCurrentPosition()
                        , motorBackRight.getCurrentPosition());
                telemetry.addData("Running to", "%7d:%7d", motorBackLeft.getCurrentPosition()
                        , motorFrontRight.getCurrentPosition());
                telemetry.addData("Currently at", "%7d:%7d",
                        motorFrontLeft.getCurrentPosition()
                        , motorBackRight.getCurrentPosition());
                telemetry.addData("Currently at", "%7d:%7d",
                        motorFrontRight.getCurrentPosition()
                        , motorBackLeft.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motorBackLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorFrontLeft.setPower(0);
            motorBackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            deadWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            resetEncoders();
        }
    }

    public void armEncoder(double pose, double speed, int timeOut, boolean isUp) {
        int target;
        target = (int) pose;
        sparkLong.setTargetPosition(target);
        sparkLong.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        if (isUp) {
            sparkLong.setPower(speed);//go down
        }
        if (!isUp) {
            sparkLong.setPower(-speed);
        }
        while (opModeIsActive() &&
                (runtime.seconds() < timeOut) && sparkLong.isBusy()) {

            // Display it for the driver.
            telemetry.addData("Running to", sparkLong.getCurrentPosition());
            telemetry.addData("Currently at",
                    sparkLong.getCurrentPosition());
            telemetry.update();
        }
        sparkLong.setPower(0);
        sparkLong.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.update();
    }

    public void runVu(int timeoutS, boolean giveSpot) {
        runtime.reset();
        while (opModeIsActive() && (spot == 0)) {
            if (runtime.seconds() > timeoutS) {
                spot = 4;
            }
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display image position/size information for each one
                    // Note: "Image number" refers to the randomized image orientation/number
                    for (Recognition recognition : updatedRecognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2;
                        double row = (recognition.getTop() + recognition.getBottom()) / 2;
                        double width = Math.abs(recognition.getRight() - recognition.getLeft());
                        double height = Math.abs(recognition.getTop() - recognition.getBottom());

                        telemetry.addData("", " ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                        telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                        if (giveSpot && spot == 0) {
                            if (Objects.equals(recognition.getLabel(), "led")) {
                                spot += 1;
                                break;
                            }
                            if (Objects.equals(recognition.getLabel(), "resistor")) {
                                spot += 2;
                                break;
                            }
                            if (Objects.equals(recognition.getLabel(), "capacitor")) {
                                spot += 3;
                                break;
                            }
                        }
                    }
                    telemetry.update();
                }
            }
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
