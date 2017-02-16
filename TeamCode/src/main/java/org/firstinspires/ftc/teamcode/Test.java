package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Libs.GAMEPAD;

/**
 * Created by edidi on 05.11.2016.
 */

@TeleOp(name = "Test", group = "Gamepad")
@Disabled
public class Test extends OpMode {
    GAMEPAD GAMEPAD1 = null;
    GAMEPAD GAMEPAD2 = null;
    Servo servoOpritor = null;
    @Override
    public void init() {
        GAMEPAD1 = new GAMEPAD(this.gamepad1, this.telemetry);
        GAMEPAD2 = new GAMEPAD(this.gamepad2, this.telemetry);
        servoOpritor = this.hardwareMap.servo.get("servoOpritor");

        servoOpritor.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void loop() {
        telemetry.addData("trigger:", GAMEPAD1.right_trigger);
        servoOpritor.setPosition(GAMEPAD1.right_trigger);
    }
}
