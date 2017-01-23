package org.firstinspires.ftc.teamcode;

import android.graphics.Path;
import android.telecom.TelecomManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Libs.GAMEPAD;

/**
 * Created by edidi on 16.10.2016.
 */
@TeleOp(name = "ShowGamepadValue", group = "Gamepad")
public class ShowGamepadValues extends OpMode{
    GAMEPAD GAMEPAD1 = null;
    GAMEPAD GAMEPAD2 = null;

    @Override
    public void init() {
        GAMEPAD1 = new GAMEPAD(this.gamepad1, this.telemetry);
        GAMEPAD2 = new GAMEPAD(this.gamepad2, this.telemetry);
    }

    @Override
    public void loop() {

        //GAMEPAD 1

        telemetry.addData("GAMEPAD1.left_stick_angle: ", GAMEPAD1.left_stick_angle);
        telemetry.addData("GAMEPAD1.right_stick_angle: ", GAMEPAD1.right_stick_angle);

        telemetry.addData("GAMEPAD1.left_stick_y", GAMEPAD1.left_stick_y);
        telemetry.addData("GAMEPAD1.left_stick_x", GAMEPAD1.left_stick_x);

        telemetry.addData("GAMEPAD1.right_stick_y", GAMEPAD1.right_stick_y);
        telemetry.addData("GAMEPAD1.right_stick_x", GAMEPAD1.right_stick_x);

        telemetry.addData("GAMEPAD1.a", GAMEPAD1.a.value);
        telemetry.addData("GAMEPAD1.b", GAMEPAD1.b.value);
        telemetry.addData("GAMEPAD1.x", GAMEPAD1.x.value);
        telemetry.addData("GAMEPAD1.y", GAMEPAD1.y.value);

        telemetry.addData("GAMEPAD1.dpad_up", GAMEPAD1.dpad_up);
        telemetry.addData("GAMEPAD1.dpad_down", GAMEPAD1.dpad_down);
        telemetry.addData("GAMEPAD1.dpad_left", GAMEPAD1.dpad_left);
        telemetry.addData("GAMEPAD1.dpad_right", GAMEPAD1.dpad_right);

        telemetry.addData("GAMEPAD1.right_bumper", GAMEPAD1.right_bumper);
        telemetry.addData("GAMEPAD1left_bumper", GAMEPAD1.left_bumper);

        telemetry.addData("GAMEPAD1.right_trigger", GAMEPAD1.right_trigger);
        telemetry.addData("GAMEPAD1.left_trigger", GAMEPAD1.left_trigger);

        //GAMEPAD 2

        telemetry.addData("GAMEPAD2.left_stick_angle: ", GAMEPAD2.left_stick_angle);
        telemetry.addData("GAMEPAD2.right_stick_angle: ", GAMEPAD2.right_stick_angle);

        telemetry.addData("GAMEPAD2.left_stick_y", GAMEPAD2.left_stick_y);
        telemetry.addData("GAMEPAD2.left_stick_x", GAMEPAD2.left_stick_x);

        telemetry.addData("GAMEPAD2.right_stick_y", GAMEPAD2.right_stick_y);
        telemetry.addData("GAMEPAD2.right_stick_x", GAMEPAD2.right_stick_x);

        telemetry.addData("GAMEPAD2.a", GAMEPAD2.a.value);
        telemetry.addData("GAMEPAD2.b", GAMEPAD2.b.value);
        telemetry.addData("GAMEPAD2.x", GAMEPAD2.x.value);
        telemetry.addData("GAMEPAD2.y", GAMEPAD2.y.value);

        telemetry.addData("GAMEPAD2.dpad_up", GAMEPAD2.dpad_up);
        telemetry.addData("GAMEPAD2.dpad_down", GAMEPAD2.dpad_down);
        telemetry.addData("GAMEPAD2.dpad_left", GAMEPAD2.dpad_left);
        telemetry.addData("GAMEPAD2.dpad_right", GAMEPAD2.dpad_right);

        telemetry.addData("GAMEPAD2.right_bumper", GAMEPAD2.right_bumper);
        telemetry.addData("GAMEPAD2.left_bumper", GAMEPAD2.left_bumper);

        telemetry.addData("GAMEPAD2.right_trigger", GAMEPAD2.right_trigger);
        telemetry.addData("GAMEPAD2.left_trigger", GAMEPAD2.left_trigger);
    }
}
