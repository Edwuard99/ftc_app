package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Libs.GAMEPAD;

/**
 * Created by edidi on 12.01.2017.
 */

@TeleOp(name = "TankBot", group = "TeleOp")
public class TankBot extends OpMode {
    private GAMEPAD GAMEPAD1 = null;
    private GAMEPAD GAMEPAD2 = null;

    private Drive drive = null;
    private Mechanisms mechanisms = null;

    @Override
    public void init() {
        GAMEPAD1 = new GAMEPAD(this.gamepad1, this.telemetry);
        GAMEPAD2 = new GAMEPAD(this.gamepad2, this.telemetry);
        drive = new Drive(this.hardwareMap, GAMEPAD1, "tank");
        mechanisms = new Mechanisms(this.hardwareMap, GAMEPAD2, this.telemetry, "tank");
    }

    @Override
    public void loop() {
        drive.goTeleOp();
        mechanisms.work();
    }

}
