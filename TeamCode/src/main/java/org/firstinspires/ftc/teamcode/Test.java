package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libs.GAMEPAD;

/**
 * Created by edidi on 05.11.2016.
 */

@TeleOp(name = "Test", group = "Gamepad")
public class Test extends OpMode {
    GAMEPAD GAMEPAD1 = null;
    GAMEPAD GAMEPAD2 = null;
    @Override
    public void init() {
        GAMEPAD1 = new GAMEPAD(this.gamepad1, this.telemetry);
        GAMEPAD2 = new GAMEPAD(this.gamepad2, this.telemetry);
    }

    @Override
    public void loop() {
        telemetry.addData("y:", GAMEPAD1.left_stick_y);

    }
}
