package org.firstinspires.ftc.teamcode.externalHardware;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "lambadaTest", group = "Robot")
@Disabled
public class lambadaTest extends LinearOpMode {
    HardwareConfig r = new HardwareConfig(this);
    GamepadEx driverOp = new GamepadEx(gamepad1);
    GamepadEx toolOp = new GamepadEx(gamepad2);
    Button sPre = new GamepadButton(
            toolOp, GamepadKeys.Button.X
    );
    Button mPre = new GamepadButton(
            toolOp, GamepadKeys.Button.B
    );
    Button tPre = new GamepadButton(
            toolOp, GamepadKeys.Button.Y
    );
    Button slo = new GamepadButton(
            driverOp, GamepadKeys.Button.RIGHT_BUMPER
    );

    @Override
    public void runOpMode() throws InterruptedException {
        r.init(hardwareMap);
        while (opModeIsActive()) {//while the op mode is active
            r.drive(false, r.slowPower);
            slo.toggleWhenPressed(new InstantCommand(this::slo));
            //sPre.whenPressed(new InstantCommand(this::sPre));
            //mPre.whenPressed(new InstantCommand(this::mPre));
            //tPre.whenPressed(new InstantCommand(this::tPre));
            r.power();
        }
    }

    public void slo() {
        if (r.slowModeIsOn) {
            r.slowPower = r.slowMult;
        } else {
            r.slowPower = 1;
        }
    }

    //public void sPre() {
    //    if (r.sparkLong.getCurrentPosition() > HardwareConfig.lowPoleVal + 50) {//go down
    //        r.armPower = -1;
    //    }
    //    if (r.sparkLong.getCurrentPosition() < HardwareConfig.lowPoleVal - 50) {//go up
    //        r.armPower = 1;
    //    }
    //}
    //
    //public void mPre() {
    //    if (r.sparkLong.getCurrentPosition() > HardwareConfig.midPoleVal + 50) {//go down
    //        r.armPower = -1;
    //    }
    //    if (r.sparkLong.getCurrentPosition() < HardwareConfig.midPoleVal - 50) {//go up
    //        r.armPower = 1;
    //    }
    //}
    //
    //public void tPre() {
    //    if (r.sparkLong.getCurrentPosition() < HardwareConfig.topPoleVal) {//go up
    //        r.armPower = 1;
    //    }
    //}
}