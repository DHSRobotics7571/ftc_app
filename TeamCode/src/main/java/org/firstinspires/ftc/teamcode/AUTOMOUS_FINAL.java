package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Autonomous ffs", group="Autonomous")
public class AUTOMOUS_FINAL extends OpMode {
    int robostate = 0;
    double throttlecontrol = 1;

    DcMotor motorRightFront,motorRightBack,motorLeftFront,motorLeftBack;
    DcMotor motorCatapult;

    Servo servoFeeder;

    public void init(){
        motorRightFront = hardwareMap.dcMotor.get("rightfront");
        //motorRightBack = hardwareMap.dcMotor.get("rightback");
        motorLeftFront = hardwareMap.dcMotor.get("leftfront");
        //motorLeftBack = hardwareMap.dcMotor.get("leftback");
        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        servoFeeder = hardwareMap.servo.get("servoFeeder");
    }

    @Override
    public void loop() {
        //3656 ticks to firing plane
        switch (robostate) {
            case 1:
            {

                robostate++;
                break;
        }
                case 2: {

                    if (!motorLeftFront.isBusy()) {
                        setThrottle(0);
                        robostate++;
                    }
                }

           case 3:
                motorCatapult.setTargetPosition(1120);
                motorCatapult.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorCatapult.setPower(1);
                if (!motorCatapult.isBusy()){
                    servoFeeder.setPosition(1);
                }
                robostate++;
                break;

            case 4:
                motorCatapult.setMode(DcMotor.RunMode.RESET_ENCODERS);
                motorCatapult.setTargetPosition(1120);
                motorCatapult.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorCatapult.setPower(1);
                robostate++;
                break;
            case 5:
                if (!motorCatapult.isBusy()){
                    motorCatapult.setPower(0);
                }

        }

    }
    public void setThrottle(double throttle){
        motorLeftBack.setPower(throttle);
        motorLeftFront.setPower(throttle);
        motorRightBack.setPower(throttle);
        motorRightFront.setPower(throttle);
    }
}
