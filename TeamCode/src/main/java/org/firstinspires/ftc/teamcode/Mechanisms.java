package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Libs.GAMEPAD;

/**
 * Created by Iedi on 13.01.2017.
 */

class Mechanisms {
    public HardwareMap hardwareMap = null;

    public GAMEPAD gamepad = null;

    Telemetry telemetry = null;
    String type = null;

    public DcMotor motorLansatorStanga = null;
    public DcMotor motorLansatorDreapta = null;

    public DcMotor motorMaturica = null;
    public DcMotor motorGlisiera = null;

    public Double power = 0.25;

    public Double powerMaturica = 0.3d;

    public Servo servoGlisiera = null;
    public Servo servoOpritor = null;

    public Servo servoApasatorStanga = null;
    public Servo servoApasatorDreapta = null;

    Mechanisms(HardwareMap hardwareMap, GAMEPAD gamepad, Telemetry telemetry, String type){
        this.gamepad = gamepad;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.type = type;

        init();

    }

    void init(){
        if(type.compareToIgnoreCase("omni") == 0)
            initOmni();
        if(type.compareToIgnoreCase("tank") == 0)
            initTank();

    }

    void initOmni(){
        power = 0.18;
        motorLansatorStanga = this.hardwareMap.dcMotor.get("motorLansatorStanga");
        motorLansatorDreapta = this.hardwareMap.dcMotor.get("motorLansatorDreapta");

        motorMaturica = this.hardwareMap.dcMotor.get("motorMaturica");
        motorGlisiera = this.hardwareMap.dcMotor.get("motorGlisiera");

        servoGlisiera = this.hardwareMap.servo.get("servoGlisiera");
        servoOpritor = this.hardwareMap.servo.get("servoOpritor");

        servoApasatorStanga = this.hardwareMap.servo.get("servoApasatorStanga");
        servoApasatorDreapta = this.hardwareMap.servo.get("servoApasatorDreapta");


        motorLansatorStanga.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLansatorDreapta.setDirection(DcMotorSimple.Direction.REVERSE);

        motorGlisiera.setDirection(DcMotorSimple.Direction.REVERSE);

        servoOpritor.setDirection(Servo.Direction.REVERSE);
        servoApasatorStanga.setDirection(Servo.Direction.REVERSE);
        servoApasatorDreapta.setDirection(Servo.Direction.FORWARD);

        motorMaturica.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    void initTank(){
        power = 0.23;
        motorLansatorStanga = this.hardwareMap.dcMotor.get("motorLansatorStanga");
        motorLansatorDreapta = this.hardwareMap.dcMotor.get("motorLansatorDreapta");

        motorMaturica = this.hardwareMap.dcMotor.get("motorMaturica");
        motorGlisiera = this.hardwareMap.dcMotor.get("motorGlisiera");

        servoGlisiera = this.hardwareMap.servo.get("servoGlisiera");
        servoOpritor = this.hardwareMap.servo.get("servoOpritor");

        servoApasatorStanga = this.hardwareMap.servo.get("servoApasatorStanga");
        servoApasatorDreapta = this.hardwareMap.servo.get("servoApasatorDreapta");


        motorLansatorStanga.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLansatorDreapta.setDirection(DcMotorSimple.Direction.REVERSE);

        servoOpritor.setDirection(Servo.Direction.FORWARD);
        servoApasatorStanga.setDirection(Servo.Direction.FORWARD);
        servoApasatorDreapta.setDirection(Servo.Direction.REVERSE);

        motorMaturica.setDirection(DcMotorSimple.Direction.FORWARD);
        motorGlisiera.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    void work(){
        powerMaturica = getPowerMaturica();

        motorLansatorStanga.setPower(gamepad.y.toggleInt * power);
        motorLansatorDreapta.setPower(gamepad.y.toggleInt * power);

        motorMaturica.setPower(powerMaturica);
        motorGlisiera.setPower(dpad_power());

        servoApasatorDreapta.setPosition(gamepad.right_bumper.toggleInt);
        servoApasatorStanga.setPosition(gamepad.left_bumper.toggleInt);

        servoGlisiera.setPosition(gamepad.left_trigger);
        servoOpritor.setPosition(gamepad.a.toggleInt * 0.9);

        telemetry.addData("lansator power: ", power);
    }

    double dpad_power(){
        double dpad_power = 0.1;
        if(gamepad.dpad_up.value == true){
            dpad_power = 1;
        }
        else if (gamepad.dpad_down.value == true){
            dpad_power = -1;
        }
        else{
            dpad_power = 0.1;
        }

        return dpad_power;
    }

    Double getPowerMaturica(){
        Double power;
        if(gamepad.b.value == true){
            power =  -1d;
        }
        else{
            power = gamepad.right_trigger;
        }
        return power;
    }
}
