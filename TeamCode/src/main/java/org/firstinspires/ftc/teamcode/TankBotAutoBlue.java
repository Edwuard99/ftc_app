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
import com.qualcomm.robotcore.util.Util;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Libs.GAMEPAD;
import org.firstinspires.ftc.teamcode.Libs.Utils;

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
    double lansatorPower = 0.2;

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

        servoApasatorStanga.setDirection(Servo.Direction.FORWARD);
        servoApasatorDreapta.setDirection(Servo.Direction.REVERSE);

        servoOpritor.setDirection(Servo.Direction.FORWARD);

        motorStangaSpate.setDirection(DcMotorSimple.Direction.REVERSE);
        motorStangaFata.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDreaptaSpate.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDreaptaFata.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLansatorDreapta.setDirection(DcMotorSimple.Direction.REVERSE);

        servoApasatorDreapta.setPosition(0);
        servoApasatorStanga.setPosition(0);

        resetEncoders();
        if(voltangeSensor.getVoltage() > 13.5) lansatorPower = 0.2;
        servoOpritor.setPosition(0);
        gyroSensor.calibrate();
        waitSec(1);
        while(gyroSensor.isCalibrating());
        gyroSensor.resetZAxisIntegrator();
        telemetry.addData("gyro calibrat","");
        telemetry.update();
        waitForStart();
        gyroSensor.resetZAxisIntegrator();
        motorLansare(lansatorPower);
        servoOpritor.setPosition(0.9);
        go(1400,"f",0.5);
        waitSec(0.2);
        //gyroPID(0);
        motorMaturica.setPower(1);
        waitSec(3);
        motorLansare(0);
        motorMaturica.setPower(0);



        gyroPID(58);

        waitSec(0.5);

        go(1800,"nobrake",0.8);
        goToWhiteLine();

        gyroPID(90);
        goToWall(5);

        motorPower(0.5,0.5);
        waitSec(0.3);
        motorPower(0,0);
        waitSec(0.1);
        apasa("blue");

        goBack(300,"break",0.2);
        gyroPID(0);
        go(1100,"nobrake",0.7);
        goToWhiteLine();

        gyroPID(90);
        goToWall(10);
        motorPower(0.2,0.2);
        waitSec(0.3);
        motorPower(0,0);
        waitSec(0.1);
        apasa("blue");

       /* goBack(400,"brake",0.3);
        goToDegr(55);
        goBack(4000,"brake",0.8);*/


        while (opModeIsActive()) {

            //motorPower(gamepad1.left_stick_y, gamepad1.right_stick_y);

            telemetry.addData("DF pos: ", motorDreaptaFata.getCurrentPosition());
            telemetry.addData("gyro", getHeading());
            telemetry.addData("gyro speed: ", gyroSensor.rawZ());
            telemetry.addData("color: ", getColor());
            telemetry.update();
            motorPower(0,0);
        }
    }

    public void gyroPID2(int target){
        double kp = 0.5;
        double ki= 0.0001;
        double kd = 0.5;
        double direction = 1;
        double lastDirection = 1;
        double P;
        double I = 0;
        double D;
        double output;
        double lasterror = 0;
        double error = 0;
        boolean targetAchieved = false;

        while(!targetAchieved){
            error = Math.abs(target - getHeading());
            P = Math.abs(error);
            P = Utils.range(P, 0d, 90d, 0d, 1d);


            D = Utils.range((double) Math.abs(gyroSensor.rawZ()), 0d, 5000d, 1d, -1d);



            P = P * kp;
            I = I + Utils.range(error, 0d, 10d, 0d, 1d) * ki;
            I = Utils.cut(I, 0d, 0.2d);
            D = D * kd;
            if(target - getHeading() <= 0){
                direction = -1;
            }
            else{
                direction = 1;
            }

            /*if (lasterror == P) {
                I += 0.0008;
            } else I -= 0.0006;

            if(I < 0)
                I = 0;

            if(lastDirection != direction)
                I = 0;*/

            output = P + I;
            output = output * direction;

            motorPower(output, -output);
            /*lasterror = P;
            if(Math.abs(target - getHeading()) < 2) I = 0;*/
            telemetry.addData("gyro: ", getHeading());
            telemetry.addData("gyro speed: ", gyroSensor.rawZ());
            telemetry.addData("P: ", P);
            telemetry.addData("D: ", D);
            telemetry.addData("I: ", I);
            telemetry.addData("direction: ", direction);
            telemetry.addData("output: ", output);
            telemetry.update();
            if(Math.abs(target - getHeading()) < 1 && gyroSensor.rawZ() < 15){
                targetAchieved = true;
            }
            waitSec(0.01);
            //lastDirection = direction;
        }

        motorPower(0, 0);

    }

    public void gyroPID(int target){
        double kp = 0.002;
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
                if (lasterror == error) {
                    I += 0.0012;
                } else I -= 0.0012;

                if (I < 0)
                    I = 0;
            }
            else{
                I = 0;
            }

            if(Math.abs(target - getHeading()) < 3) I = 0;
            output = P + I;
            output = output * direction;

            motorPower(output, -output);

            telemetry.addData("gyro: ", getHeading());
            telemetry.addData("gyro speed: ", gyroSensor.rawZ());
            telemetry.addData("P: ", P);
            telemetry.addData("D: ", D);
            telemetry.addData("I: ", I);
            telemetry.addData("direction: ", direction);
            telemetry.addData("error: ", error);
            telemetry.addData("output: ", output);
            telemetry.update();
            if(Math.abs(target - getHeading()) < 3 && gyroSensor.rawZ() < 50){
                targetAchieved = true;
            }
            waitSec(0.1);
            lastDirection = direction;
            lasterror = error;
        }

        motorPower(0, 0);
        waitSec(0.3);
    }

    public void goToWall(int cm) {
        motorPower(0.3, 0.3);
        while(rangeDreapta.cmUltrasonic() > cm);
        motorPower(0,0);
    }

    private void goToWhiteLine() {
        motorPower(0.2,0.2);
        runtime.reset();
        while(colorBurta.alpha() < 10 && runtime.seconds() < 4) {

        }
        motorPower(0d,0d);
        waitSec(0.1);
    }

    private void resetEncoders() {
        motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorStangaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorStangaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDreaptaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void go (int ticks, String brake, double speed) {
        int pos = motorDreaptaFata.getCurrentPosition();
        ticks = ticks + pos;
        motorPower(speed, speed);
        while(pos < ticks) {
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
        while(pos > ticks) {
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
        motorStangaFata.setPower(stg);
        motorDreaptaSpate.setPower(drt);
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
            while(getHeading() < (target - 20 - targetCorrection)) {
                showGyroDebug();
            }
            motorPower(0.05, -0.05);
            while(getHeading() < (target - targetCorrection)) {
                showGyroDebug();
            }
            motorPower(0, 0);
        }else{
            motorPower(-voltagePower, voltagePower);
            while(getHeading() > (target + 20 + targetCorrection)) {
                showGyroDebug();
            }
            motorPower(-0.05, 0.05);
            while(getHeading() > (target + targetCorrection)) {
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
        while (runtime.seconds() < seconds) showGyroDebug();
    }

    public void runToPosition(int ticks) {
        motorMode(DcMotor.RunMode.RUN_TO_POSITION);
        ticks = motorDreaptaFata.getCurrentPosition() + ticks;
        setTargetPosition(ticks);
        motorPower(0.7, 0.7);
        while(isBusy());
        motorPower(0, 0);
        motorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runToPositionCoast(int ticks) {
        motorMode(DcMotor.RunMode.RUN_TO_POSITION);
        ticks = motorDreaptaFata.getCurrentPosition() + ticks;
        setTargetPosition(ticks);
        motorPower(0.7, 0.7);
        while(isBusy());
        motorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //motorPower(0, 0);

    }

    public void runToPositionBack(int ticks) {
        motorMode(DcMotor.RunMode.RUN_TO_POSITION);
        ticks = motorDreaptaFata.getCurrentPosition() - ticks;
        setTargetPosition(ticks);
        motorPower(-0.7, -0.7);
        while(isBusy());
        //motorPower(0, 0);
        motorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public boolean isBusy(){
        return motorStangaFata.isBusy() || motorStangaSpate.isBusy() || motorDreaptaFata.isBusy() || motorDreaptaSpate.isBusy();
    }

    public void motorMode(DcMotor.RunMode runmode){
        motorStangaFata.setMode(runmode);
        motorStangaSpate.setMode(runmode);
        motorDreaptaFata.setMode(runmode);
        motorDreaptaSpate.setMode(runmode);
    }

    public void setTargetPosition(int ticks){
        motorStangaFata.setTargetPosition(ticks);
        motorStangaSpate.setTargetPosition(ticks);

        motorDreaptaFata.setTargetPosition(ticks);
        motorDreaptaSpate.setTargetPosition(ticks);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior){
        motorStangaFata.setZeroPowerBehavior(zeroPowerBehavior);
        motorStangaSpate.setZeroPowerBehavior(zeroPowerBehavior);

        motorDreaptaFata.setZeroPowerBehavior(zeroPowerBehavior);
        motorDreaptaSpate.setZeroPowerBehavior(zeroPowerBehavior);
    }
}
