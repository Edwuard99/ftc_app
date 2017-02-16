package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Iedi on 18.01.2017.
 */

@TeleOp(name = "ServoTest", group = "MUITA")
@Disabled
public class ServoTest extends OpMode{
    Servo servo = null;
    @Override
    public void init() {
        servo = this.hardwareMap.servo.get("servo");
    }

    @Override
    public void loop() {
        if(gamepad1.x == true)
            servo.setPosition(0);
        if(gamepad1.b == true)
            servo.setPosition(1);
    }
}
