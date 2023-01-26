package org.firstinspires.ftc.teamcode.pure;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
//@Disabled
public class PurePursuitSample extends CommandOpMode {

    // define our constants
    static final double TRACKWIDTH = 13;
    static final double WHEEL_DIAMETER = 3.89;    // inches
    static double TICKS_TO_INCHES;
    static final double CENTER_WHEEL_OFFSET = 2.4;

    private HolonomicOdometry m_robotOdometry;
    private OdometrySubsystem m_odometry;
    private PurePursuitCommand ppCommand;
    private MecanumDrive m_robotDrive;
    private Motor fL, fR, bL, bR;
    private MotorEx leftEncoder, rightEncoder, centerEncoder;

    @Override
    public void initialize() {
        fL = new Motor(hardwareMap, "motorFrontLeft");
        fR = new Motor(hardwareMap, "motorFrontRight");
        bL = new Motor(hardwareMap, "motorBackLeft");
        bR = new Motor(hardwareMap, "motorBackRight");

        // create our drive object
        m_robotDrive = new MecanumDrive(fL, fR, bL, bR);

        bL.setInverted(true);
        fL.setInverted(true);

        //leftEncoder = new MotorEx(hardwareMap, "leftEncoder");
        //rightEncoder = new MotorEx(hardwareMap, "rightEncoder");
        centerEncoder = new MotorEx(hardwareMap, "deadWheel");

        // calculate multiplier
        TICKS_TO_INCHES = WHEEL_DIAMETER * Math.PI / centerEncoder.getCPR();
        double TICKS_TO_INCHES_POWERED = WHEEL_DIAMETER * Math.PI / fL.getCPR();

        // create our odometry object and subsystem
        m_robotOdometry = new HolonomicOdometry(
                () -> fL.getCurrentPosition() * TICKS_TO_INCHES_POWERED,
                () -> bR.getCurrentPosition() * TICKS_TO_INCHES_POWERED,
                () -> centerEncoder.getCurrentPosition() * TICKS_TO_INCHES,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );
        m_odometry = new OdometrySubsystem(m_robotOdometry);

        // create our pure pursuit command
        ppCommand = new PurePursuitCommand(
                m_robotDrive, m_odometry,
                new StartWaypoint(0, 0),
                //new GeneralWaypoint(2, 0, 0.8, 0.8, 30),
                new EndWaypoint(
                        0, 50, 0, 0.5,
                        0, 0, 0.8, 1
                )

        );

        // schedule the command
        schedule(ppCommand);
    }

}