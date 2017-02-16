package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;

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

@Autonomous(name = "TankBotAutonomous", group = "Autonomous")
@Disabled
public class TankBotAutonomous extends LinearOpMode {
    DcMotor motorStangaSpate = null;
    DcMotor motorStangaFata = null;

    DcMotor motorDreaptaSpate = null;
    DcMotor motorDreaptaFata = null;

    GyroSensor gyroSensor = null;

    GAMEPAD GAMEPAD1 = null;
    //Drive drive = null;
    @Override
    public void runOpMode() throws InterruptedException {
        GAMEPAD1 = new GAMEPAD(this.gamepad1, this.telemetry);
        //drive = new Drive(this.hardwareMap, GAMEPAD1, "tank");

        gyroSensor = this.hardwareMap.gyroSensor.get("gyro");

        motorStangaSpate = hardwareMap.dcMotor.get("motorStangaSpate");
        motorStangaFata = hardwareMap.dcMotor.get("motorStangaFata");

        motorDreaptaSpate = hardwareMap.dcMotor.get("motorDreaptaSpate");
        motorDreaptaFata = hardwareMap.dcMotor.get("motorDreaptaFata");

        gyroSensor.calibrate();
        while(gyroSensor.isCalibrating());

        motorStangaFata.setDirection(DcMotorSimple.Direction.REVERSE);
        motorStangaSpate.setDirection(DcMotorSimple.Direction.REVERSE);

        motorDreaptaFata.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDreaptaSpate.setDirection(DcMotorSimple.Direction.FORWARD);

        motorStangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorStangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorDreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorStangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorStangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorDreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorDreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorStangaFata.setTargetPosition(2162);
        motorStangaSpate.setTargetPosition(2162);
        motorDreaptaFata.setTargetPosition(2162);
        motorDreaptaSpate.setTargetPosition(2162);

        motorStangaFata.setPower(0.47);
        motorStangaSpate.setPower(0.47);
        motorDreaptaFata.setPower(0.5);
        motorDreaptaSpate.setPower(0.5);

        while(Math.abs(motorStangaFata.getCurrentPosition() - 2162) < 100)

        motorStangaFata.setTargetPosition(2702);
        motorStangaSpate.setTargetPosition(2702);
        motorDreaptaFata.setTargetPosition(2702);
        motorDreaptaSpate.setTargetPosition(2702);

        motorStangaFata.setPower(0.47);
        motorStangaSpate.setPower(0.47);
        motorDreaptaFata.setPower(0.5);
        motorDreaptaSpate.setPower(0.5);

        while(Math.abs(motorStangaFata.getCurrentPosition() - 2162) < 100)

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

        waitForStart();
        while(opModeIsActive()){
            telemetry.update();
            //goCm(60);

        }
    }

    public void goCm(int cm){
        int ticks = (int)11.187 * cm;
        setMotorTicks(ticks);
        while(Math.abs(getCurrentPosition() - ticks) > 10);
    }

    public void setMotorTicks(int ticks){
        motorStangaSpate.setTargetPosition(ticks);
        motorDreaptaSpate.setTargetPosition(ticks);
    }
    public int getCurrentPosition(){
        int position = motorStangaSpate.getCurrentPosition()
                + motorDreaptaSpate.getCurrentPosition();
        position = position /2;

        return position;

    }


   /* public void goToDegr(double targetPosition){
        double target = targetPosition;
        double turnigSpeed = 0;
        double error = Math.abs(getHeading() - target);
        error = error / 180;
        while(error * 180 > 3) {
            error = Math.abs(getHeading() - target);
            error = error / 180;
            //double correctionSpeed = gyroSensor.getRotationFraction();
            turnigSpeed = error;
        }


    }*/
}
