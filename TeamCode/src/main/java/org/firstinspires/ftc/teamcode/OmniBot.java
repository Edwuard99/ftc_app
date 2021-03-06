package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Libs.GAMEPAD;

/**
 * Created by edidi on 09.10.2016.
 */

@TeleOp(name = "OmniBot", group = "TeleOp")//init
public class OmniBot extends OpMode {
    private GAMEPAD GAMEPAD1 = null;
    private GAMEPAD GAMEPAD2 = null;

    private Drive drive = null;
    private Mechanisms mechanisms = null;

    @Override
    public void init() {//takes gamepad values
        GAMEPAD1 = new GAMEPAD(this.gamepad1, this.telemetry);
        GAMEPAD2 = new GAMEPAD(this.gamepad2, this.telemetry);
        drive = new Drive(this.hardwareMap, GAMEPAD1, "omni");
        mechanisms = new Mechanisms(this.hardwareMap, GAMEPAD2, this.telemetry, "omni");
    }

    @Override
    public void loop() {//gamepad valoare
        drive.goTeleOp();
        mechanisms.work();
        telemetry.addData("gamepad y: ", GAMEPAD1.left_stick_powerY);
        telemetry.addData("gamepad x: ", GAMEPAD1.left_stick_powerX);
        telemetry.addData("gamepad y: ", gamepad1.left_stick_y);
        telemetry.addData("gamepad x: ", gamepad1.left_stick_x);

        telemetry.addData("gamepad y: ", GAMEPAD1.right_stick_powerY);
        telemetry.addData("gamepad x: ", GAMEPAD1.right_stick_powerX);
        telemetry.addData("gamepad y: ", gamepad1.right_stick_y);
        telemetry.addData("gamepad x: ", gamepad1.right_stick_x);

    }

}
