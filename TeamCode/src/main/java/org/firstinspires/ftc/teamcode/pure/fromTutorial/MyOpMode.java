package org.firstinspires.ftc.teamcode.pure.fromTutorial;

import static org.firstinspires.ftc.teamcode.pure.fromTutorial.RobotMovement.followCurve;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;

public class MyOpMode extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        ArrayList<CurvePoint> allPoints = new ArrayList<>();
        allPoints.add(new CurvePoint(0, 0, 1, 0.8, 50, 50, 0, 0));
        //add more of ^^^
        followCurve(allPoints, 90);
    }
}
