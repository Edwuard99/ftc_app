package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Util;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Libs.GAMEPAD;
import org.firstinspires.ftc.teamcode.Libs.Utils;

/**
 * Created by edidi on 1/13/2017.
 */

@Autonomous(name = "TankBotAutoBlue", group = "Autonomous")

public class TankBotAutoBlue extends LinearOpMode {
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
    Servo servoOpritor = null;
    GAMEPAD GAMEPAD1 = null;
    OpticalDistanceSensor odsLeft;
    OpticalDistanceSensor odsRight;


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
        servoOpritor = this.hardwareMap.servo.get("servoOpritor");

        rangeDreapta = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distantaDreapta");
        rangeStanga = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distantaStanga");
        rangeStanga.setI2cAddress(I2cAddr.create8bit(0x3e));

        odsLeft = hardwareMap.opticalDistanceSensor.get("odsLeft");
        odsRight = hardwareMap.opticalDistanceSensor.get("odsRight");

        servoApasatorStanga.setDirection(Servo.Direction.FORWARD);
        servoApasatorDreapta.setDirection(Servo.Direction.REVERSE);

        servoOpritor.setDirection(Servo.Direction.FORWARD);

        motorStangaSpate.setDirection(DcMotorSimple.Direction.REVERSE);
        motorStangaFata.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDreaptaSpate.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDreaptaFata.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLansatorDreapta.setDirection(DcMotorSimple.Direction.REVERSE);

        motorStangaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorStangaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDreaptaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servoApasatorDreapta.setPosition(0);
        servoApasatorStanga.setPosition(0);



        resetEncoders();
        servoOpritor.setPosition(0);
        gyroSensor.calibrate();
        waitSec(1);
        while(gyroSensor.isCalibrating());
        gyroSensor.resetZAxisIntegrator();
        telemetry.addData("gyro calibrat","");
        telemetry.update();

        waitForStart();
        motorLansare(0.25);
        servoOpritor.setPosition(0.9);
        go(1400,"f",0.6);
        waitSec(0.2);
        motorMaturica.setPower(1);
        waitSec(2);
        motorLansare(0);
        motorMaturica.setPower(0);

        gyroPID(55);

        go(2700,"nobrake",0.7);
        goToWhiteLine(0.2);

        gyroPID(90);
        goToWall(5);

        motorPower(0.5,0.5);
        waitSec(0.3);
        motorPower(0,0);
        waitSec(0.1);
        apasa("blue");

        goBack(300,"break",0.5);
        gyroPID(0);
        go(2700,"nobrake",0.7);
        goToWhiteLine(0.15);

        gyroPID(90);
        goToWall(10);
        motorPower(0.2,0.2);
        waitSec(0.5);
        motorPower(0,0);
        waitSec(0.1);
        apasa("blue");

        goBack(400,"brake",0.3);
        gyroPID(45);
        goBack(4000,"brake",0.8);

        while (opModeIsActive()) {
            telemetry.addData("odsLeft Raw: ", odsLeft.getRawLightDetected());
            telemetry.addData("odsLeft: ", odsLeft.getLightDetected());
            telemetry.addData("odsRight Raw: ", odsRight.getRawLightDetected());
            telemetry.addData("odsRight: ", odsRight.getLightDetected());
           /* telemetry.addData("gyro", getHeading());
            telemetry.addData("DF pos: ", motorDreaptaFata.getCurrentPosition());
            telemetry.addData("DS pos: ", motorDreaptaSpate.getCurrentPosition());
            telemetry.addData("SF pos: ", motorStangaFata.getCurrentPosition());
            telemetry.addData("SS pos: ", motorStangaSpate.getCurrentPosition());
            telemetry.addData("gyro speed: ", gyroSensor.rawZ());
            telemetry.addData("color: ", getColor());*/
            telemetry.update();
            motorPower(0, 0);
        }
    }

    public void slowWhite(double speed) {
        motorPower(speed, speed);
        while(odsLeft.getRawLightDetected() < 2 && odsRight.getRawLightDetected() < 2 && opModeIsActive()) ;
    }

    public void gyroPID(int target){
        double kp = 0.0035;
        double ki= 1;
        double kd = 0.2;
        double direction = 1;
        double lastDirection = 1;
        double P;
        double I = 0;
        double D;
        double output;
        double lasterror = 0;
        double error = 0;
        boolean targetAchieved = false;

        while(!targetAchieved  && opModeIsActive()){
            error = Math.abs(target - getHeading());
            //P = Utils.range(P, 0d, 90d, 0d, 1d);

            //I = Utils.range((double) Math.abs(target - getHeading()), 0d, 10d, 0d, 1d);

            D = Utils.range((double) Math.abs(gyroSensor.rawZ()), 0d, 15000d, 1d, -1d);


            //I = I + I * ki;
            P = error * kp;
            //P = Utils.cut(P, 0d, 1d);
            D = D * kd;
            if(target - getHeading() <= 0){
                direction = -1;
            }
            else{
                direction = 1;
            }

            if(error < 10) {
                if (lasterror == error) {
                    I += 0.0015;
                } else I -= 0.0015;

                if (I < 0)
                    I = 0;
            }
            else{
                I = 0;
            }

            if(Math.abs(target - getHeading()) <= 1) I = 0;
            output = P + I;
            output = output * direction;

            motorPower(output, -output);

            telemetry.addData("gyro: ", getHeading());
            telemetry.addData("gyro speed: ", gyroSensor.rawZ());
            telemetry.addData("P: ", P);
            telemetry.addData("I: ", I);
            telemetry.addData("output: ", output);
            telemetry.update();
            if(Math.abs(target - getHeading()) <= 1 && gyroSensor.rawZ() < 50){
                targetAchieved = true;
            }
            waitSec(0.1);
            lastDirection = direction;
            lasterror = error;
        }

        motorPower(0, 0);
        waitSec(0.1);
    }

    public void goToWall(int cm) {
        motorPower(0.5, 0.5);
        while(rangeDreapta.cmUltrasonic() > cm  && opModeIsActive());
        motorPower(0,0);
    }

    private void goToWhiteLine(double speed) {
        motorPower(speed,speed);
        runtime.reset();
        while(colorBurta.alpha() < 8 && runtime.seconds() < 4  && opModeIsActive()) {

        }
        motorPower(0d,0d);
        waitSec(0.1);
    }

    private void resetEncoders() {
        motorLansatorDreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLansatorStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void go (int ticks, String brake, double speed) {
        int pos = motorDreaptaFata.getCurrentPosition();
        ticks = ticks + pos;
        motorPower(speed, speed);
        while(pos < ticks  && opModeIsActive()) {
            pos = motorDreaptaFata.getCurrentPosition();
            telemetry.addData("gyro", getHeading());
            telemetry.update();
        }
        if(brake != "nobrake") {
            motorPower(0,0);
            waitSec(0.1);
        }
    }

    public void goBack (int ticks, String brake, double speed) {
        int pos = motorDreaptaFata.getCurrentPosition();
        ticks =pos - ticks;
        motorPower(-speed, -speed);
        while(pos > ticks  && opModeIsActive()) {
            pos = motorDreaptaFata.getCurrentPosition();
            telemetry.addData("gyro", getHeading());
            telemetry.update();
        }
        if(brake != "nobrake") {
            motorPower(0,0);
            waitSec(0.1);
        }
    }

    public void motorPower(double stg, double drt) {
        motorStangaSpate.setPower(stg);
        motorDreaptaSpate.setPower(drt);
        motorStangaFata.setPower(stg);
        motorDreaptaFata.setPower(drt);
    }

    public void motorLansare(double speed) {
        motorLansatorDreapta.setPower(speed);
        motorLansatorStanga.setPower(speed);
    }



    public void apasa(String color){
        if(getColor().compareToIgnoreCase("blue") == 0){
            if(color.compareToIgnoreCase("blue") == 0) {
                servoApasatorDreapta.setPosition(1);
                servoApasatorStanga.setPosition(0);
            }
            else{
                servoApasatorDreapta.setPosition(0);
                servoApasatorStanga.setPosition(1);
            }
        }
        else{
            if(color.compareToIgnoreCase("blue") == 0) {
                servoApasatorDreapta.setPosition(0);
                servoApasatorStanga.setPosition(1);
            }
            else{
                servoApasatorDreapta.setPosition(1);
                servoApasatorStanga.setPosition(0);
            }
        }
        waitSec(1);
        servoApasatorDreapta.setPosition(0);
        servoApasatorStanga.setPosition(0);

    }

    public String getColor(){
        if(colorSensor.blue() > colorSensor.red())
            return "blue";
        else
            return "red";
    }

    public void goToDegr(int target) {
        double targetCorrection;
        double correctionPower;
        int error = 2;
        if(voltangeSensor.getVoltage() > 12) {
            voltagePower = 0.2;
            targetCorrection = 15;
        }
        else {
            voltagePower = 0.3;
            targetCorrection = 0;
        }

        showGyroDebug();
        if(target - getHeading() >= 0){
            motorPower(voltagePower, -voltagePower);
            while(getHeading() < (target - 20 - targetCorrection) && opModeIsActive()) {
                showGyroDebug();
            }
            motorPower(0.05, -0.05);
            while(getHeading() < (target - targetCorrection) && opModeIsActive()) {
                showGyroDebug();
            }
            motorPower(0, 0);
        }else{
            motorPower(-voltagePower, voltagePower);
            while(getHeading() > (target + 20 + targetCorrection) && opModeIsActive()) {
                showGyroDebug();
            }
            motorPower(-0.05, 0.05);
            while(getHeading() > (target + targetCorrection) && opModeIsActive()) {
                showGyroDebug();
            }
            motorPower(0, 0);
        }
    }

    public int getHeading() {
        int heading = gyroSensor.getHeading();
        if (heading > 180)
            heading = heading - 360;
        return heading;
    }

    public void showGyroDebug() {
       /* telemetry.addData("gyro", getHeading());
        telemetry.addData("gyro speed", gyroSensor.rawZ());
        telemetry.update();

        System.out.println("gyro: " + getHeading());
        System.out.println("gyro speed" + gyroSensor.rawZ());*/
    }

    public void  waitSec(double seconds) {
        runtime.reset();
        while (runtime.seconds() < seconds  && opModeIsActive()) showGyroDebug();
    }

}
