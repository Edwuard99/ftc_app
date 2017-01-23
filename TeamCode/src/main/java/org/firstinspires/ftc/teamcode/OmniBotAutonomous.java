package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Libs.GAMEPAD;

/**
 * Created by User on 20/01/2017.
 */

@Autonomous(name = "OmniBotAutonomous", group = "pula")
public class OmniBotAutonomous extends LinearOpMode {
    int distance = 0;

    Drive drive = null;
    Mechanisms mechanisms = null;
    GAMEPAD GAMEPAD1 = null;
    GAMEPAD GAMEPAD2 = null;
    @Override
    public void runOpMode() throws InterruptedException {
        GAMEPAD1 = new GAMEPAD(this.gamepad1, this.telemetry);
        GAMEPAD2 = new GAMEPAD(this.gamepad2, this.telemetry);
        drive = new Drive(this.hardwareMap, GAMEPAD1, "omni");
        mechanisms = new Mechanisms(this.hardwareMap, GAMEPAD2, this.telemetry, "omni");


        waitForStart();

        drive.motorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.motorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(Math.abs(drive.motorA.getCurrentPosition() - 5000) < 10){
            drive.goOmni(1d, 0d, 0d);
        }
        drive.goOmni(0d, 0d, 0d);

        /*mechanisms.motorLansatorStanga.setPower(0.4);
        mechanisms.motorLansatorDreapta.setPower(0.4);

        wait(3000);

        mechanisms.servoOpritor.setPosition(1);
        wait(750);
        mechanisms.motorMaturica.setPower(1);

        wait(5000);
        mechanisms.motorLansatorStanga.setPower(0);
        mechanisms.motorLansatorDreapta.setPower(0);
        mechanisms.servoOpritor.setPosition(0);
        mechanisms.motorMaturica.setPower(0);*/



        while(opModeIsActive()){
            //telemetry.addData("motorA", drive.motorA.getCurrentPosition());
            telemetry.addData("motorA", drive.motorA.getCurrentPosition());
            //telemetry.addData("motorC", drive.motorC.getCurrentPosition());
            //telemetry.addData("motorD", drive.motorD.getCurrentPosition());
            telemetry.addData("motorA busy", drive.motorA.isBusy());
            telemetry.update();
        }
    }

    public void goCm(int cm){
        int ticks = (int)8.75 * cm / 2;


        distance = drive.motorA.getCurrentPosition() - drive.motorB.getCurrentPosition() + drive.motorC.getCurrentPosition() - drive.motorD.getCurrentPosition();
        distance = distance / 4;

        drive.motorA.setTargetPosition(ticks);
        drive.motorB.setTargetPosition(-ticks);
        drive.motorC.setTargetPosition(-ticks);
        drive.motorD.setTargetPosition(ticks);




        /*drive.motorA.setMaxSpeed(1000);
        drive.motorB.setMaxSpeed(1000);
        drive.motorC.setMaxSpeed(1000);
        drive.motorD.setMaxSpeed(1000);*/

        drive.goOmni(1d, 0d, 0d);

        while(drive.motorA.isBusy() && drive.motorB.isBusy() && drive.motorC.isBusy() && drive.motorD.isBusy());
        telemetry.addData("muie", "damian");
        telemetry.update();


        //while(Math.abs(distance - ticks) > 10);

        /*drive.motorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.motorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.motorC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.motorD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/
    }

    /*public void setMotorTicks(int ticks){
        //motorStangaSpate.setTargetPosition(ticks);
        //motorDreaptaSpate.setTargetPosition(ticks);
    }*/
    /*public int getCurrentPosition(){
        /*int position = motorStangaSpate.getCurrentPosition()
                + motorDreaptaSpate.getCurrentPosition();
        position = position /2;

        return position;

    }*/

}
