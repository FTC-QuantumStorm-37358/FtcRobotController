import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
const base='../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/quantumstorm/';
const read=file=>readFileSync(base+file,'utf8');
const adapter=read('Auto/AutoStateMachine.java');
for(const file of ['RedTeamOneAutonomous','RedTeamTwoAutonomous','BlueTeamOneAutonomous','BlueTeamTwoAutonomous']){
 assert.match(read(`Auto/${file}.java`),/new AutoStateMachine\(this, AutoStartPosition\.(RED|BLUE)_[12]\)/);
}
assert.match(adapter,/position==AutoStartPosition.RED_2\|\|position==AutoStartPosition.BLUE_2\?Routine.TEAM_TWO:Routine.TEAM_ONE/);
assert.match(adapter,/field.roleStart\(alliance,selectedRoutine==Routine.TEAM_TWO\)/);
assert.match(adapter,/AutoStateMachine\(io,field,false,selectedRoutine,false,false\)/,'Real role enables mechanisms, needs no vision, and skips final optional collection');
assert.match(adapter,/false,-1,Vision.empty/,'No fabricated measured inventory');
assert.match(adapter,/shooter.apply\(o.shooterPower,o.feederPower\)/);
assert.match(adapter,/intake.setPower\(o.intakePower\*Constants.INTAKE_POWER\)/);
assert.match(adapter,/finally\{routine.stop\(\);\}/,'Always stop on OpMode exit');
assert.match(adapter,/routine.parkingToleranceInches=2\.0/,'Hardware parking accepts position without chasing a final heading');
assert.match(adapter,/routine.waitUntilParkTime=true/,'Hardware must wait until 26 seconds before parking');
const shooter=read('Mechanisms/Shooter.java');
assert.equal((shooter.match(/hardwareMap.get\(DcMotor.class,/g)||[]).length,1,'Only one shooter motor may be required');
assert.match(shooter,/shooter.setPower\(Constants.SHOOTER_1_POWER\*scale\)/);
assert.match(shooter,/SafeShutdown.run\(\(\)->shooter.setPower\(0\),\(\)->feeder.setPower\(0\)\)/);
console.log('QuantumStorm hardware entry/role/start, live single-shooter/intake wiring and shutdown integration checks passed (not a physical hardware test).');
