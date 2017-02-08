package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.firstinspires.ftc.teamcode.Libs.GAMEPAD;
import org.firstinspires.ftc.teamcode.Libs.Utils;

/**
 * Created by edidi on 14.10.2016.
 */
public class Drive{
    public GAMEPAD gamepad = null;

    public DcMotor motorA = null;
    public DcMotor motorB = null;
    public DcMotor motorC = null;
    public DcMotor motorD = null;

    public DcMotor motorStangaFata = null;
    public DcMotor motorStangaSpate = null;
    public DcMotor motorDreaptaFata = null;
    public DcMotor motorDreaptaSpate = null;

    public String type = null;

    Drive(HardwareMap hardwareMap, GAMEPAD gamepad, String type){
        this.type = type;
        //this.gyro = hardwareMap.gyroSensor.get("gyro");

        //gyro.calibrate();
        //gyro.resetZAxisIntegrator();

        this.gamepad = gamepad;

        if(type.compareToIgnoreCase("omni") == 0) {
            initOmni(hardwareMap);
        }
        if(type.compareToIgnoreCase("tank") == 0){
            initTank(hardwareMap);
        }
        if(type.compareToIgnoreCase("mecanum") == 0){

        }
    }
    Drive(HardwareMap hardwareMap, String type){
        this.type = type;
        //this.gyro = hardwareMap.gyroSensor.get("gyro");

        //gyro.calibrate();
        //gyro.resetZAxisIntegrator();

        //this.gamepad = gamepad;

        if(type.compareToIgnoreCase("omni") == 0) {
            initOmni(hardwareMap);
        }
        if(type.compareToIgnoreCase("tank") == 0){
            initTank(hardwareMap);
        }
        if(type.compareToIgnoreCase("mecanum") == 0){

        }
    }


    private void initOmni(HardwareMap hardwareMap){
        this.motorA = hardwareMap.dcMotor.get("motorA");
        this.motorB = hardwareMap.dcMotor.get("motorB");
        this.motorC = hardwareMap.dcMotor.get("motorC");
        this.motorD = hardwareMap.dcMotor.get("motorD");

        motorA.setDirection(DcMotorSimple.Direction.FORWARD);
        motorB.setDirection(DcMotorSimple.Direction.FORWARD);
        motorC.setDirection(DcMotorSimple.Direction.FORWARD);
        motorD.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void initTank(HardwareMap hardwareMap){
        motorStangaSpate = hardwareMap.dcMotor.get("motorStangaSpate");
        motorStangaFata = hardwareMap.dcMotor.get("motorStangaFata");

        motorDreaptaSpate = hardwareMap.dcMotor.get("motorDreaptaSpate");
        motorDreaptaFata = hardwareMap.dcMotor.get("motorDreaptaFata");

        motorStangaFata.setDirection(DcMotorSimple.Direction.REVERSE);
        motorStangaSpate.setDirection(DcMotorSimple.Direction.REVERSE);

        motorDreaptaFata.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDreaptaSpate.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void setPower(double a, double b, double c, double d){
        motorA.setPower(a);
        motorB.setPower(b);
        motorC.setPower(c);
        motorD.setPower(d);
    }

    void goTeleOp(){
        if(type.compareToIgnoreCase("omni") == 0) {
            goOmni(gamepad.left_stick_powerY, gamepad.left_stick_powerX, gamepad. right_stick_powerX);
        }
        if(type.compareToIgnoreCase("tank") == 0){
            goTank(gamepad.left_stick_powerY, gamepad.right_stick_powerY);
        }
        if(type.compareToIgnoreCase("mecanum") == 0){

        }
    }

    public void goTank(Double power_left, Double power_right){
        if(gamepad.y.toggle == false) {
            if (gamepad.a.toggle == true) {
                motorStangaFata.setPower(power_left);
                motorStangaSpate.setPower(power_left);

                motorDreaptaFata.setPower(power_right);
                motorDreaptaSpate.setPower(power_right);
            } else {
                motorStangaFata.setPower(power_left * 0.3);
                motorStangaSpate.setPower(power_left * 0.3);

                motorDreaptaFata.setPower(power_right * 0.3);
                motorDreaptaSpate.setPower(power_right * 0.3);
            }
        }
        else{
            if (gamepad.a.toggle == true) {
                motorStangaFata.setPower(-power_right);
                motorStangaSpate.setPower(-power_right);

                motorDreaptaFata.setPower(-power_left);
                motorDreaptaSpate.setPower(-power_left);
            } else {
                motorStangaFata.setPower(-power_right * 0.3);
                motorStangaSpate.setPower(-power_right * 0.3);

                motorDreaptaFata.setPower(-power_left * 0.3);
                motorDreaptaSpate.setPower(-power_left * 0.3);
            }

        }
    }
    public void goOmni(Double left_stick_powerY, Double left_stick_powerX, Double right_stick_powerX){
        double a = left_stick_powerY + left_stick_powerX;
        double b = -left_stick_powerY + left_stick_powerX;
        double c = -left_stick_powerY - left_stick_powerX;
        double d = left_stick_powerY - left_stick_powerX;

        a += right_stick_powerX;
        b += right_stick_powerX;
        c += right_stick_powerX;
        d += right_stick_powerX;

        a = Utils.cut(a, -1d, 1d);
        b = Utils.cut(b, -1d, 1d);
        c = Utils.cut(c, -1d, 1d);
        d = Utils.cut(d, -1d, 1d);

        if (gamepad.a.toggle == false) {
            a = a/4;
            b = b/4;
            c = c/4;
            d = d /4;
        }

        setPower(a, b, c, d);
    }

    public void goOmniAutonomous(Double left_stick_powerY, Double left_stick_powerX, Double right_stick_powerX){
        double a = left_stick_powerY + left_stick_powerX;
        double b = -left_stick_powerY + left_stick_powerX;
        double c = -left_stick_powerY - left_stick_powerX;
        double d = left_stick_powerY - left_stick_powerX;

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

}
