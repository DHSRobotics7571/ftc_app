package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Autonomous ffs", group="Autonomous")
public class AUTOMOUS_FINAL extends OpMode {
    int robostate = 0;
    double throttlecontrol = 1;

    DcMotor motorRightFront,motorRightBack,motorLeftFront,motorLeftBack;
    DcMotor motorLinearRight,motorLinearLeft;
    DcMotor motorFeeder;
    DcMotor motorCatapult;

    Servo servoFeeder;

    public void init(){
        motorRightFront = hardwareMap.dcMotor.get("rightfront");
        motorRightBack = hardwareMap.dcMotor.get("rightback");
        motorLeftFront = hardwareMap.dcMotor.get("leftfront");
        motorLeftBack = hardwareMap.dcMotor.get("leftback");
        motorLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        servoFeeder = hardwareMap.servo.get("servofeeder");


    }
    public void loop() {
        //3656 ticks to firing plane
        switch (robostate){
            case 1:

                motorLeftBack.setTargetPosition(3656);
                motorLeftBack.RunMode.RUN_TO_POSITION();
                setThrottle(1);
                robostate++;
                break;

         /*   case 2:

                if (!motorLeftFront.isBusy()){
                    setThrottle(0);
                }
*/

        }

    }
    public void setThrottle(double throttle){
        motorLeftBack.setPower(throttle);
        motorLeftFront.setPower(throttle);
        motorRightBack.setPower(throttle);
        motorRightFront.setPower(throttle);
    }
}
