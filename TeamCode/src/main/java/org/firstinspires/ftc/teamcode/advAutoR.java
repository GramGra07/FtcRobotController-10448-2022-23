package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.teleOp.scrap;

import java.util.List;
import java.util.Objects;

@Autonomous(name = "advAutoR", group = "Robot")
//@Disabled
public class advAutoR extends scrap {
    public int turn = 77;


    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 28;
    static final double WHEEL_DIAMETER_MM = 96;
    static final double WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_MM * 0.0393701;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * 15) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double COUNTS_PER_MOTOR_REV_dead = 8192;
    static final double WHEEL_DIAMETER_MM_dead = 96;
    static final double WHEEL_DIAMETER_INCHES_dead = WHEEL_DIAMETER_MM_dead * 0.0393701;     // For figuring circumference
    static final double COUNTS_PER_INCH_dead = (COUNTS_PER_MOTOR_REV_dead) /
            (WHEEL_DIAMETER_INCHES_dead * Math.PI);

    static final double ROBOT_DIAMETER = 13.05;
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
    public double IN_distanceR = 0;//in distance for distance sensor 1
    public double IN_distanceL = 0;
    public double myMagic = 7;
    private DigitalChannel red2;
    private DigitalChannel green2;
    //color
    final float[] hsvValues = new float[3];//gets values for color sensor
    private float redVal = 0;//the red value in rgb
    private float greenVal = 0;//the green value in rgb
    private float blueVal = 0;//the blue value in rgb
    private String colorName = "N/A";//gets color name
    NormalizedColorSensor colorSensor;//declaring the colorSensor variable
    public TouchSensor touchSensor;
    public boolean touchPressed = false;
    public int ovrCurrX = 2;
    public int ovrCurrY = 1;
    private final boolean bypassCam = true;
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
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        deadWheel = hardwareMap.get(DcMotor.class, "deadWheel");
        deadWheelL = hardwareMap.get(DcMotor.class, "deadWheelL");
        deadWheelR = hardwareMap.get(DcMotor.class, "deadWheelR");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        sparkLong = hardwareMap.get(DcMotor.class, "sparkLong");
        touchSensor = hardwareMap.get(TouchSensor.class, ("touchSensor"));

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
        red2.setMode(DigitalChannel.Mode.OUTPUT);
        green2.setMode(DigitalChannel.Mode.OUTPUT);

        telemetry.addData("Starting at", "%7d :%7d",
                motorBackRight.getCurrentPosition(),
                motorBackLeft.getCurrentPosition(),
                motorFrontLeft.getCurrentPosition());
        closeClaw();
        if (!bypassCam) {
            initVuforia();
            initTfod();
            if (tfod != null) {
                tfod.activate();
                tfod.setZoom(1.0, 16.0 / 9.0);
            }
            runVu(6, false);
        }
        telemetry.update();
        closeClaw();
        // Wait for the game to start (driver presses PLAY)
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        waitForStart();
        if (opModeIsActive()) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.valueOf(getColor()));
            final int rotation = 92;
            final double coneSubtraction = (fiveTallConeVal * 0.2);
            if (!bypassCam) {
                runVu(6, true);
            }
            if (spot == 0) {
                spot = (int) (Math.floor(Math.random() * (3 - 1 + 1) + 1));
            }
            //branch 1
            advGoSpot(ovrCurrX, ovrCurrY, 2.1, 3.2 ,0.8, false, topPoleVal, false,
                    "|", 1, true, -rotation, false, false, null, 0,
                    false, null, 0, false);
                        //by now should be at pole, facing it with arm extended to top
            //branch 2
            armEncoder(topPoleVal,1,3,false);
            sideWaysEncoderDrive(1,-1,0.5);
            openClaw();
            //branch 3
            double halfTile = 3.0;
            turn(12);
            closeClaw();
            sideWaysEncoderDrive(1, halfTile, 2);
            //should now be lined up with the cone stack
            int stackDist = 36;//primary distance to go to stack
            encoderComboFwd(0.8, stackDist, stackDist, midPoleVal, 5, true);
            ////!using color
            //getAllColor();
            ////red colorInRange(redVal,0.36, greenVal,.25, blueVal,.16, (float) 0.05);
            ////blue colorInRange(redVal,0.12, greenVal,.27, blueVal,.47, (float) 0.05);
            //getAllColor();
            //boolean correctR = colorInRange(redVal,0.36, greenVal,.25, blueVal,.16, (float) 0.05);
            //boolean correctB = colorInRange(redVal,0.12, greenVal,.27, blueVal,.47, (float) 0.05);
            //while (!correctR || !correctB) {
            //
            //    getAllColor();
            //    correctR = colorInRange(redVal,0.36, greenVal,.25, blueVal,.16, (float) 0.05);
            //    correctB = colorInRange(redVal,0.12, greenVal,.27, blueVal,.47, (float) 0.05);
            //}
            //!using touch
            correct();
            armEncoder(fiveTallConeVal+100,1,3,true);
            openClaw();
            armEncoder(fiveTallConeVal,1,3,true);
            closeClaw();
            armEncoder(midPoleVal+500, 1, 3, false);//clear gap
            //branch 4
            //now has cone ready for next placement
            int repetitions = 1;
            double finished = 0;
            halfTile=-6;
            stackDist=24;
            //!not finished from here on
            while (repetitions >= 1) {
                encoderComboFwd(0.8, -stackDist, -stackDist, topPoleVal, 6, false);//back up
                sideWaysEncoderDrive(1, halfTile, 3);
                openClaw();
                sideWaysEncoderDrive(1, -halfTile, 1);
                turn(-10);
                closeClaw();
                encoderComboFwd(1.0, stackDist, stackDist, midPoleVal+500, 4, true);//should be at cone stack after this
                correct();
                armEncoder(fiveTallConeVal - (coneSubtraction * finished),1,3,true);
                openClaw();
                sleep(300);
                closeClaw();
                // gets every pole val 5tall-((928/5)*finished poles)
                armEncoder(midPoleVal+500, 1, 1, false);
                repetitions -= 1;
                finished += 1;
            }
            encoderComboFwd(1, -stackDist, -stackDist, baseArmPosition, 3, true);//get to 2,3
            stackDist=14;
            if (spot == 3) {
                encoderDrive(1, stackDist,stackDist, 3);//opposite of 3 lines higher
            }
            //should already be here at spot 2
            if (spot == 2) {
            }
            if (spot == 1) {
                encoderDrive(1, -stackDist,-stackDist, 3);
            }
            telemetry.update();
        }
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
    public void correct(){
        boolean pressed = touchSensor.isPressed();
        while (!pressed) {
            pressed = touchSensor.isPressed();
            if (pressed) {
                break;
            }
            encoderDrive(1, 3, 3, 1);
        }
    }
    public void getAllColor() {
        //gives color values
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        getColorRGB(colors.red, colors.green, colors.blue);
        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue)
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2])
                .addData("Alpha", "%.3f", colors.alpha);
        telemetry.addLine()
                .addData("Color", colorName)
                .addData("RGB", "(" + redVal + "," + greenVal + "," + blueVal + ")");//shows rgb value
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
}
