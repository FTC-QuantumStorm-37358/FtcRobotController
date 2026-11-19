package org.firstinspires.ftc.teamcode.quantumstorm.Mechanisms;

import java.lang.reflect.Proxy;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/** Verify API isolation and explicit run-mode reset when switching comparison TeleOps. */
public final class ShooterControlModeChecks {
    private static void check(boolean condition,String why) {
        if(!condition)throw new AssertionError(why);
    }

    public static void main(String[] args) {
        double[] output={Double.NaN};
        int[] powerCalls={0},velocityCalls={0};
        DcMotor.RunMode[] mode={null};
        MotorConfigurationType originalType=new MotorConfigurationType();
        originalType.setTicksPerRev(2786);
        originalType.setGearing(99.5);
        originalType.setMaxRPM(60);
        MotorConfigurationType[] motorType={originalType};
        DcMotorEx motor=(DcMotorEx)Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[]{DcMotorEx.class},(proxy,method,values)->{
                    switch(method.getName()) {
                        case "getMotorType": return motorType[0];
                        case "setMotorType": motorType[0]=(MotorConfigurationType)values[0];return null;
                        case "setMode": mode[0]=(DcMotor.RunMode)values[0];break;
                        case "setPower":
                            check(mode[0]==DcMotor.RunMode.RUN_WITHOUT_ENCODER,"Power uses open-loop mode");
                            powerCalls[0]++;output[0]=(double)values[0];break;
                        case "setVelocity":
                            check(mode[0]==DcMotor.RunMode.RUN_USING_ENCODER,"Velocity uses encoder mode");
                            check(values.length==2 && values[1]==AngleUnit.DEGREES,"Velocity uses angular units");
                            velocityCalls[0]++;output[0]=(double)values[0];break;
                        default: throw new AssertionError("Unexpected motor call: "+method.getName());
                    }
                    return null;
                });
        // Reuse one motor to model STOP/INIT transitions between the two OpModes.
        for(ShooterControlMode control:new ShooterControlMode[]{
                ShooterControlMode.POWER,ShooterControlMode.VELOCITY,ShooterControlMode.POWER}) {
            int oldPowerCalls=powerCalls[0],oldVelocityCalls=velocityCalls[0];
            boolean velocity=control==ShooterControlMode.VELOCITY;
            control.configure(motor);
            check(output[0]==0,"INIT clears the previous OpMode output");
            check(motorType[0]!=originalType && originalType.getTicksPerRev()==2786
                    && originalType.getGearing()==99.5 && originalType.getMaxRPM()==60,
                    "Shooter calibration must not change other motors' shared profile");
            check(motorType[0].getTicksPerRev()==28 && motorType[0].getGearing()==1
                    && motorType[0].getMaxRPM()==6000,"Apply the confirmed 1:1 motor specifications");
            if(velocity) {
                control.setOutput(motor,.10);
                check(Math.abs(output[0]/360*motorType[0].getTicksPerRev()-280)<1e-6,
                        "600 RPM requests 280 ticks/sec, not 27860");
                double actualRpm=280/motorType[0].getTicksPerRev()*60;
                check(actualRpm==600,"280 measured ticks/sec displays 600 RPM");
            }
            control.setOutput(motor,control.initialSetting);
            check(Math.abs(output[0]-(velocity?17640:.54))<1e-6,"Default is 2940 RPM or 54% power");
            control.setOutput(motor,control.initialSetting+control.increment);
            check(Math.abs(output[0]-(velocity?18000:.55))<1e-6,"One step is 60 RPM or .01 power");
            control.setOutput(motor,2);
            check(output[0]==(velocity?36000:1),"Positive output is bounded");
            control.setOutput(motor,-2);
            check(output[0]==(velocity?-36000:-1),"Negative output is bounded");
            for(double invalid:new double[]{Double.NaN,Double.POSITIVE_INFINITY,Double.NEGATIVE_INFINITY}) {
                control.setOutput(motor,invalid);
                check(output[0]==0,"Invalid target stops the motor");
            }
            control.setOutput(motor,control.initialSetting);
            control.setOutput(motor,0);
            check(output[0]==0,"STOP clears the active output");
            check(velocity?powerCalls[0]==oldPowerCalls:velocityCalls[0]==oldVelocityCalls,
                    "Each TeleOp uses only its selected motor command API, including INIT and STOP");
        }
        System.out.println("Power/velocity API isolation, mode switching, defaults, steps, bounds and STOP checks passed.");
    }
}
