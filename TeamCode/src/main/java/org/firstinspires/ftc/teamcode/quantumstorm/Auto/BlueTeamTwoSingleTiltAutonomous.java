package org.firstinspires.ftc.teamcode.quantumstorm.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Team Two Single Tilt",group="Quantum Storm",preselectTeleOp="Quantum Storm Two Driver")
public final class BlueTeamTwoSingleTiltAutonomous extends SingleTiltAutoBase {
    @Override protected AutoStartPosition startPosition(){return AutoStartPosition.BLUE_2;}
}
