package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Libs.GAMEPAD;

/**
 * Created by edidi on 1/13/2017.
 */

/*diametru roata = 4 in
circumferinta roata = 31.9185814cm
motor ticks = 280 ticks per rotation
motor to wheel ratio = 1.28
ticks per wheel rotation = 358.4
32 cm -- 358 ticks
0.089 cm -- 1 tick
1 cm -- 11.187 ticks
60 cm per tile
1 tile --
*/

@Autonomous(name = "TankBotAutonomousTesting", group = "Autonomous")
public class TankBotAutonomousTesting extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor motorStangaSpate = null;
    DcMotor motorStangaFata = null;

    DcMotor motorDreaptaSpate = null;
    DcMotor motorDreaptaFata = null;

    DcMotor motorMaturica = null;
    DcMotor motorLansatorStanga = null;
    DcMotor motorLansatorDreapta = null;

    Servo servoApasatorStanga = null;
    Servo servoApasatorDreapta = null;
    ColorSensor colorSensor = null;
    ColorSensor colorBurta = null;
    GyroSensor gyroSensor = null;
    VoltageSensor voltangeSensor = null;
    ModernRoboticsI2cRangeSensor rangeDreapta;
    ModernRoboticsI2cRangeSensor rangeStanga;
    public double voltagePower = 0.2;

    GAMEPAD GAMEPAD1 = null;

    //Drive drive = null;
    @Override
    public void runOpMode() {
        gyroSensor = this.hardwareMap.gyroSensor.get("gyro");

        motorStangaSpate = hardwareMap.dcMotor.get("motorStangaSpate");
        motorStangaFata = hardwareMap.dcMotor.get("motorStangaFata");

        motorDreaptaSpate = hardwareMap.dcMotor.get("motorDreaptaSpate");
        motorDreaptaFata = hardwareMap.dcMotor.get("motorDreaptaFata");

        motorMaturica = hardwareMap.dcMotor.get("motorMaturica");
        motorLansatorDreapta = hardwareMap.dcMotor.get("motorLansatorDreapta");
        motorLansatorStanga = hardwareMap.dcMotor.get("motorLansatorStanga");

        colorSensor = this.hardwareMap.colorSensor.get("culoare");
        colorBurta = this.hardwareMap.colorSensor.get("culoareBurta");
        colorBurta.setI2cAddress(I2cAddr.create8bit(0x3a));
        voltangeSensor = this.hardwareMap.voltageSensor.get("lansator");

        servoApasatorStanga = this.hardwareMap.servo.get("servoApasatorStanga");
        servoApasatorDreapta = this.hardwareMap.servo.get("servoApasatorDreapta");

        rangeDreapta = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distantaDreapta");
        rangeStanga = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distantaStanga");
        rangeStanga.setI2cAddress(I2cAddr.create8bit(0x3e));

        servoApasatorStanga.setDirection(Servo.Direction.FORWARD);
        servoApasatorDreapta.setDirection(Servo.Direction.REVERSE);

        motorStangaSpate.setDirection(DcMotorSimple.Direction.REVERSE);
        motorStangaFata.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDreaptaSpate.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDreaptaFata.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLansatorDreapta.setDirection(DcMotorSimple.Direction.REVERSE);

        servoApasatorDreapta.setPosition(0);
        servoApasatorStanga.setPosition(0);

       // resetEncoders();
        gyroSensor.calibrate();
        while(gyroSensor.isCalibrating());
        gyroSensor.resetZAxisIntegrator();
        telemetry.addData("gyro calibrat","");
        telemetry.update();
        waitForStart();
        gyroSensor.resetZAxisIntegrator();
        //launchBall();
       /* go(700,"f",0.3);
        goToDegr(-45);

        go(2500,"nobrake",0.8);
        goToWhiteLine();

        goToDegr(-90);
        goToWall(10);

        motorPower(0.2,0.2);
        waitSec(0.15);
        motorPower(0,0);
        waitSec(0.1);
        apasa("red");

        goBack(500,"break",0.2);
        goToDegr(0);
        go(1600,"nobrake",0.7);
        goToWhiteLine();

        goToDegr(-90);
        goToWall(10);
        motorPower(0.2,0.2);
        waitSec(0.15);
        motorPower(0,0);
        waitSec(0.1);
        apasa("red");

        goBack(400,"brake",0.3);
        goToDegr(-45);
        goBack(4000,"brake",1);


        while (opModeIsActive()) {
            telemetry.addData("DF pos: ", motorDreaptaFata.getCurrentPosition());
            telemetry.addData("gyro", getHeading());
            telemetry.addData("color: ", getColor());
            telemetry.update();
        }*/
    }



}
