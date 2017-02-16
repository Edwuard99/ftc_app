package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Libs.GAMEPAD;
import org.firstinspires.ftc.teamcode.Libs.Utils;

/**
 * Created by User on 20/01/2017.
 */

@Autonomous(name = "OmniTestStraight", group = "pula")
@Disabled
public class OmniTest extends LinearOpMode {
    int distance = 0;

    Drive drive = null;
    Mechanisms mechanisms = null;
    GAMEPAD GAMEPAD1 = null;
    GAMEPAD GAMEPAD2 = null;
    ColorSensor colorBurta = null;
    GyroSensor gyroSensor = null;
    Servo servoApasatorStanga = null;
    Servo servoApasatorDreapta = null;
    ColorSensor colorSensor = null;
    ModernRoboticsI2cRangeSensor rangeDreapta;
    DcMotor motorMaturica = null;
    DcMotor motorLansatorStanga = null;
    DcMotor motorLansatorDreapta = null;
    Servo servoOpritor = null;

    public DcMotor motorA = null;
    public DcMotor motorB = null;
    public DcMotor motorC = null;
    public DcMotor motorD = null;

    public double a,b,c,d;

    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        GAMEPAD1 = new GAMEPAD(this.gamepad1, this.telemetry);
        GAMEPAD2 = new GAMEPAD(this.gamepad2, this.telemetry);
        drive = new Drive(this.hardwareMap, GAMEPAD1, "omni");
        mechanisms = new Mechanisms(this.hardwareMap, GAMEPAD2, this.telemetry, "omni");

        drive.motorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rangeDreapta = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "distantaDreapta");
        colorBurta = this.hardwareMap.colorSensor.get("culoareBurta");
        colorSensor = this.hardwareMap.colorSensor.get("culoare");
        colorSensor.setI2cAddress(I2cAddr.create8bit(0x3a));
        gyroSensor = this.hardwareMap.gyroSensor.get("gyro");

        servoOpritor = this.hardwareMap.servo.get("servoOpritor");
        servoApasatorStanga = this.hardwareMap.servo.get("servoApasatorStanga");
        servoApasatorDreapta = this.hardwareMap.servo.get("servoApasatorDreapta");

        motorMaturica = hardwareMap.dcMotor.get("motorMaturica");
        motorLansatorDreapta = hardwareMap.dcMotor.get("motorLansatorDreapta");
        motorLansatorStanga = hardwareMap.dcMotor.get("motorLansatorStanga");

        servoApasatorStanga.setDirection(Servo.Direction.FORWARD);
        servoApasatorDreapta.setDirection(Servo.Direction.REVERSE);
        servoOpritor.setDirection(Servo.Direction.FORWARD);

        servoApasatorDreapta.setPosition(0);
        servoApasatorStanga.setPosition(0);
        servoOpritor.setPosition(0);

        this.motorA = hardwareMap.dcMotor.get("motorA");
        this.motorB = hardwareMap.dcMotor.get("motorB");
        this.motorC = hardwareMap.dcMotor.get("motorC");
        this.motorD = hardwareMap.dcMotor.get("motorD");

        motorA.setDirection(DcMotorSimple.Direction.FORWARD);
        motorB.setDirection(DcMotorSimple.Direction.FORWARD);
        motorC.setDirection(DcMotorSimple.Direction.FORWARD);
        motorD.setDirection(DcMotorSimple.Direction.FORWARD);

        gyroSensor.calibrate();
        waitSec(1);
        while(gyroSensor.isCalibrating());
        gyroSensor.resetZAxisIntegrator();
        telemetry.addData("gyro calibrat","");
        telemetry.update();

        waitForStart();
        drive(0.5, 0d, 0d);
        waitSec(3);
        drive(0d, 0d, 0d);

        while(opModeIsActive()){
            telemetry.addData("range: ", rangeDreapta.cmUltrasonic());
            telemetry.addData("gyro:", getHeading());
            telemetry.addData("burta alpha: ", colorBurta.alpha());
            telemetry.addData("culoare beacon: ", getColor());
            telemetry.update();
        }
    }

    public void go (int ticks, double speed) {
        int pos = drive.motorA.getCurrentPosition();
        ticks = ticks + pos;
        drive(speed, 0d, 0d);
        while(pos < ticks) {
            pos = drive.motorA.getCurrentPosition();
            telemetry.addData("gyro", getHeading());
            telemetry.update();
        }
        drive(0d, 0d, 0d);
        waitSec(0.3);
    }

    public void motorLansare(double power){
        motorLansatorDreapta.setPower(power);
        motorLansatorStanga.setPower(power);
    }

    public void goBack (int ticks, double speed) {
        int pos = drive.motorA.getCurrentPosition();
        ticks = pos - ticks;
        drive(-speed, 0d, 0d);
        while(pos > ticks) {
            pos = drive.motorA.getCurrentPosition();
            telemetry.addData("gyro", getHeading());
            telemetry.update();
        }
        drive(0d, 0d, 0d);
    }

    public void goToWall(int cm) {
        drive.goOmniAutonomous(0.2, 0d, 0d);
        while(rangeDreapta.cmUltrasonic() > cm){
            telemetry.addData("range:", rangeDreapta.cmUltrasonic());
            telemetry.update();
        }
        drive.goOmniAutonomous(0d, 0d, 0d);
    }

    public void gyroPID(int target){
        double kp = 0.0035;
        double ki= 1;
        double kd = 0.2;
        double direction = 1;
        double lastDirection = 1;
        double P;
        double I = 0.1;
        double D;
        double output;
        double lasterror = 0;
        double error = 0;
        boolean targetAchieved = false;

        while(!targetAchieved){
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
                I = 0.05;
                if (lasterror == error) {
                    I += 0.002;
                } else I -= 0.001;

                if (I < 0)
                    I = 0;
            }
            else{
                I = 0;
            }

            if(Math.abs(target - getHeading()) < 2) I = 0;
            output = P + I;
            output = output * direction;

            drive(0d, 0d, output);

            telemetry.addData("gyro: ", getHeading());
            telemetry.addData("gyro speed: ", gyroSensor.rawZ());
            telemetry.addData("P: ", P);
            telemetry.addData("D: ", D);
            telemetry.addData("I: ", I);
            telemetry.addData("direction: ", direction);
            telemetry.addData("error: ", error);
            telemetry.addData("output: ", output);
            telemetry.update();
            if(Math.abs(target - getHeading()) < 2 && gyroSensor.rawZ() < 50){
                targetAchieved = true;
            }
            waitSec(0.1);
            lastDirection = direction;
            lasterror = error;
        }

        drive(0d, 0d, 0d);
        waitSec(0.1);
    }

    private void goToWhiteLine(String direction) {
        if(direction == "left") {
            drive.goOmniAutonomous(0d, -0.2, 0d);
            while(colorBurta.alpha() < 8) {
            }
        } else {
            drive.goOmniAutonomous(0d, 0.2, 0d);
            while(colorBurta.alpha() < 8) {
            }
        }
        drive.goOmniAutonomous(0d, 0d, 0d);
        waitSec(0.3);
    }

    public void goToDegr(int target) {
        if (target - getHeading() >= 0) {
            drive.goOmniAutonomous(0d, 0d, 0.1);
            while(Math.abs(target - getHeading()) >= 5) showGyroDebug();
            drive.goOmniAutonomous(0d, 0d, 0d);
        } else {
            drive.goOmniAutonomous(0d, 0d, -0.1);
            while(Math.abs(target - getHeading()) >= 5) showGyroDebug();
            drive.goOmniAutonomous(0d, 0d, 0d);
        }
    }

    public int getHeading() {
        int heading = gyroSensor.getHeading();
        if (heading > 180)
            heading = heading - 360;
        return heading;
    }

    public void showGyroDebug() {
        telemetry.addData("gyro", getHeading());
        telemetry.update();
    }

    public void  waitSec(double seconds) {
        runtime.reset();
        while (runtime.seconds() < seconds) debug();
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

    public void drive(Double left_stick_powerY, Double left_stick_powerX, Double right_stick_powerX){
        a = left_stick_powerY + left_stick_powerX;
        b = -left_stick_powerY + left_stick_powerX;
        c = -left_stick_powerY - left_stick_powerX;
        d = left_stick_powerY - left_stick_powerX;

        a += right_stick_powerX;
        b += right_stick_powerX;
        c += right_stick_powerX;
        d += right_stick_powerX;

        a = Utils.cut(a, -1d, 1d);
        b = Utils.cut(b, -1d, 1d);
        c = Utils.cut(c, -1d, 1d);
        d = Utils.cut(d, -1d, 1d);

        setPower(a, b, c, d);
    }

    private void debug() {

    }

    private void setPower(double a, double b, double c, double d){
        motorA.setPower(a);
        motorB.setPower(b);
        motorC.setPower(c);
        motorD.setPower(d);
    }
}
