//import
package org.firstinspires.ftc.teamcode;
import android.content.Context;
import android.graphics.Color;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardWareGrade;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.List;
import java.util.Locale;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
public class HardWareGrade {
    //motors
    public DcMotor motorFrontLeft=null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackRight = null;
    //servo
    public DcMotor sparkLong=null;
    public DcMotor sparkShort=null;
    //public Servo spear=null;

    public BNO055IMU imu;    //imu module inside expansion hub
    //push sensor
    public DigitalChannel digitalTouch;  
    // color sensor
    public NormalizedColorSensor sensor_color;
    //distance
    public DistanceSensor distance1;   //distance sensor
    public Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) distance1;//helps init distance sensors
    
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        digitalTouch = ahwMap.get(DigitalChannel.class, ("digital_touch"));
        
        sensor_color =ahwMap.get(NormalizedColorSensor.class, ("sensor_color"));
        
        distance1 = ahwMap.get(DistanceSensor.class, ("distance1"));
        
        motorFrontLeft = ahwMap.get(DcMotor.class, ( "motorBackLeft " ));
        motorBackLeft  = ahwMap.get(DcMotor.class, (" motorBackRight " ));
        motorFrontRight = ahwMap.get(DcMotor.class,("motorFrontLeft " ));
        motorBackRight = ahwMap.get(DcMotor.class,  "motorFrontRight" );
        sparkLong= ahwMap.get(DcMotor.class,("sparkShort"));
        sparkShort= ahwMap.get(DcMotor.class,("sparkLong"));
        //spear=     ahwMap.get(Servo.class, ("spear"));
        imu = ahwMap.get(BNO055IMU.class, "imu1");
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        motorFrontLeft .setPower(0);
        motorBackLeft  .setPower(0);
        motorFrontRight.setPower(0);
        motorBackRight .setPower(0);
        sparkLong.setPower(0);
        sparkShort.setPower(0);


        motorFrontLeft .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft  .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight .setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


}
