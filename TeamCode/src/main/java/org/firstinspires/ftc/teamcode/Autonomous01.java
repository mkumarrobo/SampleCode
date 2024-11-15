package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Control Hub
 * Motor Port 0 FLMotor
 * Motor Port 1 RLMotor
 * Motor Port 2 RRMotor
 * Motor Port 3 FRMotor
 *
 * I2C Port 0 RRDistance
 * I2C Port 1 FLDistance
 * I2C Port 2 FRDistance
 * I2C Port 3 RLDistance
 *
 * Extension Hub Motor 0 Extend Motor
 * Extension Hub Motor 1 Elbow Motor
 * Extension Hub Motor 2 Riser Motor
 */


@Autonomous(name="Auton1")
public class Autonomous01 extends LinearOpMode {

    DcMotor motor_FLM = null;
    DcMotor motor_RLM = null;
    DcMotor motor_FRM = null;
    DcMotor motor_RRM = null;

    DcMotor motor_Extend = null;
    DcMotor motor_Elbow = null;
    DcMotor motor_Riser = null;

    Servo motor_Gripper = null;

    DistanceSensor FRDist = null;
    DistanceSensor FLDist = null;
    DistanceSensor RLDist = null;
    DistanceSensor RRDist = null;
    DistanceSensor RightDist = null;
    DistanceSensor LeftDist = null;

    double Kp = 0.04;
    double Kd = 0.0004;
    double maxPID_Power = 0.75;
    double Kp_posn = 0.04;
    double Kd_posn = 0.006;
    double maxPID_Power_posn = 0.5;
    double Kp_strf = 0.008;
    double Kd_strf = 0.00008;
    double maxPID_Power_strf = 0.7;
    double lastError = 0;
    int hangDist = 7;
    double pickDist = 11;
    double avgFrontDist = 0;
    int extendCmd = 0;

    double distanceTol = 1;
    double power=0;
    double gripperOpenPosn = 0.99;
    double gripperSampleClosePosn = 0.1;
    double gripperClosePosn = 0;
    int autonomousStage = -1;
    int armExtensionsTol = 10;
    int armElbowHomePosn = 10;
    int armRiserHomePosn = 0;
    int straightDirection = 0;
    int backDirection = 180;
    double yaw = 0;
    double startTime = 0;

    int elbowZeroDegreeOffset = 837;
    int elbowSamplePickAngle = -20;
    double elbowCountPerDegree = 14.67;
    int extendHomePosn = 40;
    int extendStartPosn = 600;
    int elbowStartPosn = 1100;
    int elbowHangPosn = 950;
    int elbowSpecimenPickPosn = 600;
    int extendHangPosn = 450;  //-800;
    int elbowHangAngle = 0;
    boolean armPosnControl = false;
    ElapsedTime timer = new ElapsedTime();
    IMU imu;





///////////////////////////////////////////////////////////////////////////////////

    @Override
    public void runOpMode() {
        initHardware();
        distTelemetry();
        motor_Gripper.setPosition(gripperClosePosn);
        resetArmExtensions();
        timer.reset();
        while(!isStarted()){
            distTelemetry();
            if(timer.seconds()<2) {
                motor_Gripper.setPosition(0.35);
                motor_Elbow.setTargetPosition(elbowStartPosn);
                motor_Elbow.setPower(0.9);
                if(motor_Elbow.getCurrentPosition()>400) {
                    motor_Elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motor_Extend.setTargetPosition(extendStartPosn);
                    motor_Extend.setPower(0.95);
                    motor_Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }
            if(timer.seconds()>5){
                motor_Gripper.setPosition(gripperSampleClosePosn);
            }
        }
        waitForStart();
        motor_Gripper.setPosition(gripperSampleClosePosn);
        while(opModeIsActive()){
            if(autonomousStage ==-1){
                startTime = getRuntime();
                autonomousStage = 0;
            }
            if(autonomousStage==0) {
                if(getRuntime()-startTime<1.5) {
                    if (Math.abs(FLDist.getDistance(DistanceUnit.INCH) - hangDist) > distanceTol) {
                        power = PIDControl(hangDist, FLDist.getDistance(DistanceUnit.INCH), Kp_posn, Kd_posn, maxPID_Power_posn);
                        drvStraight(-power);
                    } else {
                        motor_Extend.setTargetPosition(extendStartPosn + 200);
                        motor_Elbow.setTargetPosition(elbowHangPosn);
                        drvStraight(0.3);
                        timer.reset();
                        autonomousStage = 1;
                    }
                }
                else {
                    motor_Extend.setTargetPosition(extendStartPosn + 200);
                    motor_Elbow.setTargetPosition(elbowHangPosn);
                    drvStraight(0.3);
                    timer.reset();
                    autonomousStage = 1;
                }
            }
            if(autonomousStage==1){
                if(timer.seconds()>0) {
                    drvStraight(0);
                    motor_Gripper.setPosition(gripperOpenPosn);
                    resetDriveEncoders();
                    startTime = getRuntime();
                    autonomousStage = 2;
                }
            }
            if(autonomousStage ==2) {
                if(getRuntime()-startTime>0.75){
                    armPosnControl = true;
                    motor_Extend.setTargetPosition(extendHomePosn);
                    timer.reset();
                    autonomousStage = 3;
                }
            }
            if(autonomousStage ==3) {
                if(motor_Extend.getCurrentPosition()<400){
                    autonomousStage = 4;
                    startTime = getRuntime();
                }
            }
            if(autonomousStage==4){
                if(getRuntime()-startTime<1) {
                    if (Math.min(FLDist.getDistance(DistanceUnit.INCH), FRDist.getDistance(DistanceUnit.INCH)) < 6) {
                        drvStraight(-0.3);
                    } else {
                        drvStraight(0);
                        startTime = getRuntime();
                        autonomousStage = 5;
                    }
                }
                else{
                    drvStraight(0);
                    startTime = getRuntime();
                    autonomousStage = 5;
                }
            }
            if(autonomousStage==5){
                if(getRuntime()-startTime<0.25) {
                    if (Math.abs(straightDirection - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) > 1.5) {
                        power = PIDControl(straightDirection, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), Kp, Kd, maxPID_Power);
                        pidDrive(power);
                    } else {
                        pidDrive(0);
                        resetDriveEncoders();
                        autonomousStage = 6;
                    }
                } else {
                    pidDrive(0);
                    resetDriveEncoders();
                    autonomousStage = 6;
                }
            }
            if(autonomousStage==6){
                if(Math.abs(Math.abs(motor_FLM.getCurrentPosition()) - 1450) > 20){
                    power = PIDControl(1450, Math.abs(motor_FLM.getCurrentPosition()), Kp_strf, Kd_strf, 0.8);
                    strafe(power);
                    timer.reset();
                }
                else {
                    strafe(0);
                    startTime = getRuntime();
                    autonomousStage = 7;
                }
            }
            if(autonomousStage==7){
                if(getRuntime()-startTime<0.25) {
                    if (Math.abs(straightDirection - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) > 1) {
                        power = PIDControl(straightDirection, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), Kp, Kd, maxPID_Power);
                        pidDrive(power);
                    } else {
                        pidDrive(0);
                        resetDriveEncoders();
                        autonomousStage = 8;
                    }
                } else {
                    pidDrive(0);
                    resetDriveEncoders();
                    autonomousStage = 8;
                }
            }
            if(autonomousStage==8){
                if(Math.abs(Math.abs(motor_FLM.getCurrentPosition()) - 1600) > 20){
                    power = PIDControl(1600, Math.abs(motor_FLM.getCurrentPosition()), Kp_strf, Kd_strf, 0.95);
                    drvStraight(power);
                }
                else {
                    drvStraight(0);
                    startTime = getRuntime();
                    autonomousStage = 9;
                }
            }
            if(autonomousStage==9){
                if(getRuntime()-startTime<0.25) {
                    if (Math.abs(straightDirection - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) > 1) {
                        power = PIDControl(straightDirection, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), Kp, Kd, maxPID_Power);
                        pidDrive(power);
                    } else {
                        pidDrive(0);
                        resetDriveEncoders();
                        autonomousStage = 10;
                    }
                } else {
                    pidDrive(0);
                    resetDriveEncoders();
                    autonomousStage = 10;
                }
            }
            if(autonomousStage==10){
                if(Math.abs(Math.abs(motor_FLM.getCurrentPosition()) - 425) > 20){
                    power = PIDControl(425, Math.abs(motor_FLM.getCurrentPosition()), Kp_strf, Kd_strf, maxPID_Power_strf);
                    strafe(power);
                }
                else {
                    strafe(0);
                    startTime = getRuntime();
                    autonomousStage = 11;
                }
            }
            if(autonomousStage==11){
                if(getRuntime()-startTime<0.25) {
                    if (Math.abs(straightDirection - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) > 1) {
                        power = PIDControl(straightDirection, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), Kp, Kd, maxPID_Power);
                        pidDrive(power);
                    } else {
                        pidDrive(0);
                        resetDriveEncoders();
                        motor_Elbow.setTargetPosition(elbowSpecimenPickPosn);
                        autonomousStage = 12;
                    }
                } else {
                    pidDrive(0);
                    resetDriveEncoders();
                    motor_Elbow.setTargetPosition(elbowSpecimenPickPosn);
                    autonomousStage = 12;
                }
            }
            if (autonomousStage == 12){
                if(Math.abs(Math.abs(motor_FLM.getCurrentPosition()) - 1900) > 20){
                    power = PIDControl(1900, Math.abs(motor_FLM.getCurrentPosition()), Kp_strf, Kd_strf, 0.95);
                    drvStraight(-power);
                }
                else {
                    drvStraight(0);
                    startTime = getRuntime();
                    autonomousStage = 13;
                }
            }
            if(autonomousStage ==13) {
                if (getRuntime() - startTime < 0.25) {
                    yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                    if (yaw < -150) {
                        yaw = 360 + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                    }
                    if (Math.abs(straightDirection - yaw) > 1) {
                        power = PIDControl(straightDirection, yaw, Kp, Kd, maxPID_Power);
                        pidDrive(power);
                    } else {
                        pidDrive(0);
                        resetDriveEncoders();
                        startTime = getRuntime();
                        autonomousStage = 14;
                    }
                } else {
                    pidDrive(0);
                    resetDriveEncoders();
                    startTime = getRuntime();
                    autonomousStage = 14;
                }
            }
            if(autonomousStage ==14){
                if((Math.abs(Math.abs(motor_FLM.getCurrentPosition()) - 350) > 20)&& (getRuntime()-startTime<0.75)){
                    power = PIDControl(400, Math.abs(motor_FLM.getCurrentPosition()), Kp_strf, Kd_strf, maxPID_Power_strf);
                    drvStraight(power);
                }
                else {
                    drvStraight(0);
                    startTime = getRuntime();
                    autonomousStage = 15;
                }
            }
            if(autonomousStage ==15){
                if(getRuntime()-startTime<2) {
                    yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                    if (yaw < -150) {
                        yaw = 360 + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                    }
                    if (Math.abs(backDirection - yaw) > 1) {
                        power = PIDControl(backDirection, yaw, Kp, Kd, maxPID_Power);
                        pidDrive(power);
                    } else {
                        pidDrive(0);
                        resetDriveEncoders();
                        autonomousStage = 19;
                    }
                } else {
                    pidDrive(0);
                    resetDriveEncoders();
                    autonomousStage = 19;
                }
            }
            if(autonomousStage==16){
                if(Math.abs(Math.abs(motor_FLM.getCurrentPosition()) - 500) > 20){
                    power = PIDControl(500, Math.abs(motor_FLM.getCurrentPosition()), Kp_strf, Kd_strf, 0.5);
                    strafe(-power);
                }
                else {
                    strafe(0);
                    timer.reset();
                    autonomousStage = 17;
                }
            }
            if(autonomousStage ==17){
                if(timer.seconds()<1){
                    strafe(-0.2);
                }
                else {
                    startTime = getRuntime();
                    autonomousStage = 18;
                }
            }
            if(autonomousStage ==18){
                if(getRuntime()-startTime<0.5) {
                    yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                    if (yaw < -150) {
                        yaw = 360 + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                    }
                    if (Math.abs(backDirection - yaw) > 5) {
                        power = PIDControl(backDirection, yaw, Kp, Kd, maxPID_Power);
                        pidDrive(power);
                    } else {
                        pidDrive(0);
                        startTime = getRuntime();
                        autonomousStage = 19;
                    }
                }
                else {
                    autonomousStage = 19;
                    startTime = getRuntime();
                }
            }
            if(autonomousStage==19) {
                if(getRuntime()-startTime<2.5) {
                    avgFrontDist = Math.max(FLDist.getDistance(DistanceUnit.INCH), FRDist.getDistance(DistanceUnit.INCH));
                    if (Math.abs(avgFrontDist - pickDist) > 1) {
                        power = PIDControl(pickDist, avgFrontDist, 0.1, Kd_posn, 0.35);
                        drvStraight(-power);
                    } else {
                        drvStraight(0);
                        extendCmd = (int) ((avgFrontDist - 6.5)*150 - 35)*1;
                    }
                }
                else {
                    drvStraight(0);
                    startTime = getRuntime();
                    extendCmd = (int) ((avgFrontDist - 6.5)*150 - 35)*1;
                    motor_Extend.setTargetPosition(extendCmd);
                    autonomousStage = 20;
                }
            }
            if(autonomousStage ==20){
                if(getRuntime()-startTime>0) {
                    if (motor_Extend.getCurrentPosition() < (extendCmd - 30)) {
                        motor_Gripper.setPosition(gripperClosePosn);
                        timer.reset();
                        autonomousStage = 21;
                    }
                }
            }
            if(autonomousStage == 21){
                if(timer.seconds()>1) {
                    motor_Extend.setTargetPosition(400);
                    motor_Elbow.setTargetPosition(elbowStartPosn+25);
                    drvStraight(-0.2);
                    autonomousStage = 22;
                }
            }
            if(autonomousStage == 22){
                if (getRuntime() - startTime> 0.75) {
                    drvStraight(0);
                }
                if(motor_Elbow.getCurrentPosition()>elbowSpecimenPickPosn+400) {
                    motor_Extend.setTargetPosition(extendStartPosn - 300);
                    startTime = getRuntime();
                    autonomousStage = 23;
                }
            }
            if(autonomousStage ==23){
                if(getRuntime()-startTime<1.5) {
                    if (Math.abs(straightDirection - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) > 1.5) {
                        power = PIDControl(straightDirection, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), Kp, Kd, maxPID_Power);
                        pidDrive(power);
                    } else {
                        pidDrive(0);
                        resetDriveEncoders();
                        autonomousStage = 24;
                    }
                }
                else{
                    pidDrive(0);
                    resetDriveEncoders();
                    autonomousStage = 24;
                }
            }
            if(autonomousStage == 24){
                if(Math.abs(Math.abs(motor_FLM.getCurrentPosition()) - 2800) > 20){
                    power = PIDControl(2800, Math.abs(motor_FLM.getCurrentPosition()), Kp_strf, Kd_strf, 0.95);
                    strafe(-power);
                }
                else {
                    strafe(0);
                    motor_Extend.setTargetPosition(extendStartPosn);
                    startTime = getRuntime();
                    autonomousStage = 25;
                }
            }
            if(autonomousStage ==25){
                if(getRuntime()-startTime<0.25) {
                    if (Math.abs(straightDirection - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) > 1.5) {
                        power = PIDControl(straightDirection, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), Kp, Kd, maxPID_Power);
                        pidDrive(power);
                    } else {
                        pidDrive(0);
                        resetDriveEncoders();
                        startTime = getRuntime();
                        autonomousStage = 26;
                    }
                }
                else {
                    pidDrive(0);
                    resetDriveEncoders();
                    startTime = getRuntime();
                    autonomousStage = 26;
                }
            }
            if(autonomousStage==26) {
                if(getRuntime()-startTime<1) {
                    if (Math.abs(FLDist.getDistance(DistanceUnit.INCH) - hangDist) > distanceTol) {
                        power = PIDControl(hangDist, FLDist.getDistance(DistanceUnit.INCH), Kp_posn, Kd_posn, maxPID_Power_posn);
                        drvStraight(-power);
                    } else {
                        motor_Extend.setTargetPosition(extendStartPosn);
                        motor_Elbow.setTargetPosition(elbowHangPosn);
                        startTime = getRuntime();
                        drvStraight(0.3);
                        autonomousStage = 27;
                    }
                }
                else {
                    motor_Extend.setTargetPosition(extendStartPosn);
                    motor_Elbow.setTargetPosition(elbowHangPosn);
                    drvStraight(0.3);
                    startTime = getRuntime();
                    autonomousStage = 27;
                }
            }
            if(autonomousStage==27){
                if(getRuntime()-startTime>0.5) {
                    drvStraight(0);
                    motor_Gripper.setPosition(gripperOpenPosn);
                    resetDriveEncoders();
                    autonomousStage = 28;
                }
            }
            if(autonomousStage ==28) {
                armPosnControl = true;
                motor_Extend.setTargetPosition(extendHomePosn);
                startTime = getRuntime();
                autonomousStage = 29;
            }
            if(autonomousStage ==29) {
                if(motor_Extend.getCurrentPosition()<200){
                    if(getRuntime()-startTime<2) {
                        yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                        if (yaw < -150) {
                            yaw = 360 + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                        }
                        if (Math.abs(backDirection-5 - yaw) > 1) {
                            power = PIDControl(backDirection-5, yaw, Kp, Kd, maxPID_Power);
                            pidDrive(power);
                        } else {
                            pidDrive(0);
                            resetDriveEncoders();
                            autonomousStage = 30;
                        }
                    } else {
                        pidDrive(0);
                        resetDriveEncoders();
                        autonomousStage = 30;
                    }
                }
            }
            if(autonomousStage ==30) {
                if(Math.abs(Math.abs(motor_FLM.getCurrentPosition()) - 2800) > 20){
                    power = PIDControl(2800, Math.abs(motor_FLM.getCurrentPosition()), Kp_strf, Kd_strf, 0.95);
                    strafe(-power);
                }
                else {
                    strafe(0);
                    startTime = getRuntime();
                    motor_Elbow.setTargetPosition(elbowSpecimenPickPosn);
                    autonomousStage = 31;
                }
            }
            if (autonomousStage == 31){
                if(motor_Extend.getCurrentPosition()<200){
                    if(getRuntime()-startTime<0.5) {
                        yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                        if (yaw < -150) {
                            yaw = 360 + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                        }
                        if (Math.abs(backDirection - yaw) > 1) {
                            power = PIDControl(backDirection, yaw, Kp, Kd, maxPID_Power);
                            pidDrive(power);
                        } else {
                            pidDrive(0);
                            resetDriveEncoders();
                            autonomousStage = 32;
                        }
                    } else {
                        pidDrive(0);
                        resetDriveEncoders();
                        autonomousStage = 32;
                    }
                }
            }
            if(autonomousStage ==32){
                if(getRuntime()-startTime<2.5) {
                    avgFrontDist = Math.max(FLDist.getDistance(DistanceUnit.INCH), FRDist.getDistance(DistanceUnit.INCH));
                    if (Math.abs(avgFrontDist - pickDist) > 1) {
                        power = PIDControl(pickDist, avgFrontDist, 0.1, Kd_posn, 0.35);
                        drvStraight(-power);
                    } else {
                        drvStraight(0);
                        extendCmd = (int) ((avgFrontDist - 6.5)*150 - 35)*1;
                    }
                }
                else {
                    drvStraight(0);
                    startTime = getRuntime();
                    extendCmd = (int) ((avgFrontDist - 6.5)*150 - 35)*1;
                    motor_Extend.setTargetPosition(extendCmd);
                    autonomousStage = 33;
                }
            }
            if(autonomousStage ==33){
                if(getRuntime()-startTime>0) {
                    if (motor_Extend.getCurrentPosition() < (extendCmd - 30)) {
                        motor_Gripper.setPosition(gripperClosePosn);
                        timer.reset();
                        autonomousStage = 34;
                    }
                }
            }
            if(autonomousStage==34){
                if(timer.seconds()>1) {
                    motor_Extend.setTargetPosition(400);
                    motor_Elbow.setTargetPosition(elbowStartPosn+25);
                    drvStraight(-0.2);
                    startTime = getRuntime();
                    autonomousStage = 35;
                }
            }
            if(autonomousStage ==35){
                if (getRuntime() - startTime> 0.5) {
                    drvStraight(0);
                }
                if(motor_Elbow.getCurrentPosition()>elbowSpecimenPickPosn+400) {
                    motor_Extend.setTargetPosition(extendStartPosn - 300);
                    startTime = getRuntime();
                    autonomousStage = 36;
                }
            }
            if(autonomousStage ==36){
                if(getRuntime()-startTime<1.5) {
                    if (Math.abs(straightDirection - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) > 1.5) {
                        power = PIDControl(straightDirection, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), Kp, Kd, maxPID_Power);
                        pidDrive(power);
                    } else {
                        pidDrive(0);
                        resetDriveEncoders();
                        autonomousStage = 37;
                    }
                }
                else{
                    pidDrive(0);
                    resetDriveEncoders();
                    autonomousStage = 37;
                }
            }
            if(autonomousStage==37) {
                if(Math.abs(Math.abs(motor_FLM.getCurrentPosition()) - 2600) > 20){
                    power = PIDControl(2600, Math.abs(motor_FLM.getCurrentPosition()), Kp_strf, Kd_strf, 0.95);
                    strafe(-power);
                }
                else {
                    strafe(0);
                    motor_Extend.setTargetPosition(extendStartPosn);
                    startTime = getRuntime();
                    autonomousStage = 38;
                }
            }
            if(autonomousStage ==38){
                if(getRuntime()-startTime<0.5) {
                    if (Math.abs(straightDirection - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) > 1.5) {
                        power = PIDControl(straightDirection, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), Kp, Kd, maxPID_Power);
                        pidDrive(power);
                    } else {
                        pidDrive(0);
                        resetDriveEncoders();
                        startTime = getRuntime();
                        autonomousStage = 39;
                    }
                }
                else {
                    pidDrive(0);
                    resetDriveEncoders();
                    startTime = getRuntime();
                    autonomousStage = 39;
                }
            }
            if(autonomousStage == 39){
                if(getRuntime()-startTime<1) {
                    if (Math.abs(FLDist.getDistance(DistanceUnit.INCH) - hangDist) > distanceTol) {
                        power = PIDControl(hangDist, FLDist.getDistance(DistanceUnit.INCH), Kp_posn, Kd_posn, maxPID_Power_posn);
                        drvStraight(-power);
                    } else {
                        motor_Extend.setTargetPosition(extendStartPosn);
                        motor_Elbow.setTargetPosition(elbowHangPosn);
                        startTime = getRuntime();
                        drvStraight(0.3);
                        autonomousStage = 40;
                    }
                }
                else {
                    motor_Extend.setTargetPosition(extendStartPosn);
                    motor_Elbow.setTargetPosition(elbowHangPosn);
                    drvStraight(0.3);
                    startTime = getRuntime();
                    autonomousStage = 40;
                }
            }
            if(autonomousStage == 40){
                if(getRuntime()-startTime>0.5) {
                    drvStraight(0);
                    motor_Gripper.setPosition(gripperOpenPosn);
                    autonomousStage = 41;
                }
            }
            distTelemetry();
        }
    }


///////////////////////////////////////////////////////////////////////////////////



    public void pidDrive(double dr) {
        motor_FLM.setPower(dr);
        motor_FRM.setPower(dr);
        motor_RLM.setPower(dr);
        motor_RRM.setPower(dr);
    }

    public void resetDriveEncoders(){
        motor_FLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_FRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_RLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_RRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_FLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_FRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_RLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_RRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void mecanum_drive(double dr, double st, double tw) {
//        dr*=-1;
        double[] speeds = {
                (dr + st + tw),
                (dr - st - tw),
                (dr - st + tw),
                (dr + st - tw)
        };
        double max = Math.abs(speeds[0]);
        for (int i = 0; i < speeds.length; i++) {
            if (max < Math.abs(speeds[i])) max = Math.abs(speeds[i]);
        }
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] *= 0.5/max;
        }
        motor_FLM.setPower(speeds[0]);
        motor_FRM.setPower(speeds[1]);
        motor_RLM.setPower(speeds[2]);
        motor_RRM.setPower(speeds[3]);
    }



    public double PIDControl(double refrence, double state, double p, double d, double pwr) {
        double error = (refrence - state);
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        double output = (error * p) + (derivative * d);
        output = Math.min(output, pwr);
        output = Math.max(output, -pwr);
        return output;
    }


    public void drvStraight(double pwr){
        motor_FLM.setPower(-pwr);
        motor_FRM.setPower(pwr);
        motor_RLM.setPower(-pwr);
        motor_RRM.setPower(pwr);
    }
    public void strafe(double dr){
        motor_FLM.setPower(-dr);
        motor_FRM.setPower(-dr);
        motor_RLM.setPower(dr);
        motor_RRM.setPower(dr);
    }

    public double calcStrafeDist(){
        return (motor_FLM.getCurrentPosition()+motor_FRM.getCurrentPosition()-motor_FLM.getCurrentPosition()-motor_RRM.getCurrentPosition());
    }

    public void armExtensionsHome(){
        motor_Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_Elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_Riser.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(Math.abs(motor_Extend.getCurrentPosition()-extendHomePosn)>armExtensionsTol) {
            motor_Extend.setTargetPosition(extendHomePosn);
            motor_Extend.setPower(0.9);
            motor_Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if(Math.abs(motor_Elbow.getCurrentPosition()-armElbowHomePosn)>armExtensionsTol) {
            motor_Elbow.setTargetPosition(10);
            motor_Elbow.setPower(-0.5);
            motor_Elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if(Math.abs(motor_Riser.getCurrentPosition()-armRiserHomePosn)>armExtensionsTol) {
            motor_Riser.setTargetPosition(00);
            motor_Riser.setPower(-0.35);
            motor_Riser.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void armHangSpecimenPosition(int extend, int elbow){
        motor_Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_Elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if((Math.abs(motor_Extend.getCurrentPosition()-extend)>armExtensionsTol) && armPosnControl) {
            motor_Extend.setTargetPosition(extend);
            motor_Extend.setPower(-0.5);
            motor_Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("Extend Motor Request", extend);
            telemetry.update();
        }
        if((Math.abs(motor_Elbow.getCurrentPosition()-elbow)>armExtensionsTol) && armPosnControl) {
            motor_Elbow.setTargetPosition(elbow);
            motor_Elbow.setPower(0.95);
            motor_Elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        armPosnControl = false;
    }
    public void testArmExtensions(){
        motor_Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_Elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_Riser.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_Extend.setTargetPosition(-700);
        motor_Extend.setPower(-0.75);
        motor_Extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_Elbow.setTargetPosition(1350);
        motor_Elbow.setPower(0.5);
        motor_Elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_Riser.setTargetPosition(300);
        motor_Riser.setPower(0.35);
        motor_Riser.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void resetArmExtensions(){
        startTime = getRuntime();
        motor_Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_Elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_Riser.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while((getRuntime()-startTime)<2){
            motor_Extend.setPower(-0.2);
            motor_Elbow.setPower(-0.2);
            motor_Riser.setPower(-0.9);
        }
        motor_Extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_Elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_Riser.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_Extend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_Elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void initHardware(){
        initDistanceSens();
        initDriveMotors();
        initArmExtensions();
        initServo();
        initializeIMU();
    }
    public void initDriveMotors(){
        motor_FLM = hardwareMap.dcMotor.get("FLMotor");
        motor_RLM = hardwareMap.dcMotor.get("RLMotor");
        motor_FRM = hardwareMap.dcMotor.get("FRMotor");
        motor_RRM = hardwareMap.dcMotor.get("RRMotor");
        motor_FLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_RLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_FRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_RRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void initArmExtensions(){
        motor_Extend = hardwareMap.dcMotor.get("ExtendMotor");
        motor_Elbow = hardwareMap.dcMotor.get("ElbowMotor");
        motor_Riser = hardwareMap.dcMotor.get("RiserMotor");
    }
    public void initDistanceSens(){
        FRDist = hardwareMap.get(DistanceSensor.class, "FRDistance");
        FLDist = hardwareMap.get(DistanceSensor.class, "FLDistance");
        RRDist = hardwareMap.get(DistanceSensor.class, "RRDistance");
        RLDist = hardwareMap.get(DistanceSensor.class, "RLDistance");
        RightDist = hardwareMap.get(DistanceSensor.class, "rightDistance");
        LeftDist = hardwareMap.get(DistanceSensor.class, "leftDistance");
    }
    public void initializeIMU(){
        imu=hardwareMap.get(IMU.class, "IMU");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }
    public void initServo(){
        motor_Gripper = hardwareMap.servo.get("servoGripper");
    }


    public void distTelemetry(){
        telemetry.addData("Left Distance in Inches", "%.2f", LeftDist.getDistance(DistanceUnit.INCH));
        telemetry.addData("Front Left Distance in Inches", "%.2f", FLDist.getDistance(DistanceUnit.INCH));
        telemetry.addData("Front Right Distance in Inches", "%.2f", FRDist.getDistance(DistanceUnit.INCH));
        telemetry.addData("Rear Left Distance in Inches", "%.2f", RLDist.getDistance(DistanceUnit.INCH));
        telemetry.addData("Rear Right Distance in Inches", "%.2f", RRDist.getDistance(DistanceUnit.INCH));
        telemetry.addData("Elbow Motor Encoder", motor_Elbow.getCurrentPosition());
        telemetry.addData("Extend Motor Encoder", motor_Extend.getCurrentPosition());
        telemetry.addData("Extend Motor Command", extendCmd);
//        telemetry.addData("FLM Encoder", motor_FLM.getCurrentPosition());
//        telemetry.addData("FRM Encoder", motor_FRM.getCurrentPosition());
//        telemetry.addData("RLM Encoder", motor_RLM.getCurrentPosition());
//        telemetry.addData("RRM Encoder", motor_RRM.getCurrentPosition());
//        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
//        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Autonomous Stage", autonomousStage);
//        telemetry.addData("Game Time", "%.2f", getRuntime());
        telemetry.update();
    }
}
