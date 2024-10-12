package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Control Hub
 * Motor Port 0 FLMotor
 * Motor Port 1 RLMotor
 * Motor Port 2 RRMotor
 * Motor Port 3 FRMotor
 * I2C Port 0
 * I2C Port 1
 * I2C Port 2 FRDistance
 * I2C Port 3 RLDistance
 */
@Autonomous(name="TestSens")
public class TestSensors extends LinearOpMode {

    DcMotor motor_FLM = null;
    DcMotor motor_RLM = null;
    DcMotor motor_FRM = null;
    DcMotor motor_RRM = null;

    DistanceSensor FRDist = null;
    DistanceSensor FLDist = null;
    DistanceSensor RLDist = null;
    DistanceSensor RRDist = null;

    @Override
    public void runOpMode() {
        initHardware();
        while(!isStarted()){}
        waitForStart();
        while(opModeIsActive()){
            testDrvMotors();
            distTelemetry();
        }
    }


    public void initHardware(){
        motor_FLM = hardwareMap.dcMotor.get("FLMotor");
        motor_RLM = hardwareMap.dcMotor.get("RLMotor");
        motor_FRM = hardwareMap.dcMotor.get("FRMotor");
        motor_RRM = hardwareMap.dcMotor.get("RRMotor");
        initDistanceSens();
    }

    public void initDistanceSens(){
        FRDist = hardwareMap.get(DistanceSensor.class, "FRDistance");
        FLDist = hardwareMap.get(DistanceSensor.class, "FLDistance");
        RRDist = hardwareMap.get(DistanceSensor.class, "RRDistance");
        RLDist = hardwareMap.get(DistanceSensor.class, "RLDistance");
    }

    public void testDrvMotors() {
        motor_FLM.setPower(-0.1);
        motor_RLM.setPower(-0.1);
        motor_FRM.setPower(0.1);
        motor_RRM.setPower(0.1);
    }

    public void distTelemetry(){
        telemetry.addData("Front Left Distance in CM", "%.2f", FLDist.getDistance(DistanceUnit.CM));
        telemetry.addData("Front Right Distance in CM", "%.2f", FRDist.getDistance(DistanceUnit.CM));
        telemetry.addData("Rear Left Distance in CM", "%.2f", RLDist.getDistance(DistanceUnit.CM));
        telemetry.addData("Rear Right Distance in CM", "%.2f", RRDist.getDistance(DistanceUnit.CM));
        telemetry.update();
    }
}
