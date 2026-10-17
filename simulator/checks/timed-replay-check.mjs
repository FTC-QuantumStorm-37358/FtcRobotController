import assert from 'node:assert/strict';
import {createInterface} from 'node:readline';
import {spawnSync} from 'node:child_process';
import {readFileSync} from 'node:fs';
import path from 'node:path';
import * as T from '../dist/vendor/three.module.js';
import {compileJava,launchJava,java,classes} from '../bridge/java-runtime.mjs';
import {SimActuators} from '../dist/sim-actuators.js';
import {fencePose} from '../dist/ballistics.js';
const ctx=new Proxy({},{get:()=>()=>{},set:()=>true});globalThis.document={createElement:()=>({getContext:()=>ctx})};
const {buildField,buildRobot}=await import('../dist/model.js');
const {GamePhysics,robotAllowed}=await import('../dist/game-physics.js');
await compileJava([path.resolve('robot/tests/TimedDriveChecks.java')]);
const unit=spawnSync(java,['-cp',classes,'org.firstinspires.ftc.teamcode.biobuzz.TimedDriveChecks'],{encoding:'utf8'});assert.equal(unit.status,0,unit.stdout+unit.stderr);console.log(unit.stdout.trim());
const wrapper=readFileSync('../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/quantumstorm/Auto/AutoStateMachine.java','utf8');
assert(!wrapper.includes('new PinpointOdometry')&&!wrapper.includes('odometry.update()'),'Actual robot must not initialize or query Pinpoint');
for(const alliance of [0,1])for(const singleTilt of [false,true]){
 const child=launchJava(),lines=createInterface({input:child.stdout}),queue=[],waiters=[];
 lines.on('line',line=>{const m=JSON.parse(line);if(waiters.length)waiters.shift()(m);else queue.push(m);});
 const next=()=>queue.length?Promise.resolve(queue.shift()):new Promise(resolve=>waiters.push(resolve));await next();
 const send=async m=>{child.stdin.write(JSON.stringify({protocol:1,...m})+'\n');return next();};
 try{
  const field=buildField(),robot=buildRobot(),scene=new T.Scene();scene.add(field,robot);
  const game=new GamePhysics(field,scene,robot);game.pose=fencePose(alliance);game.syncRobot();
  const actuators=new SimActuators(game,()=>assert.fail('Playback must not shoot'));
  let time=0,seq=0,parkAt=null,sawPickup=false;
  // Completely unusable odometry, disconnected camera and zero ball count are deliberate.
  const sensors={pose:{x:0,y:0,heading:0},sampledAt:-100,odometryValid:false,cameraConnected:false,ballCount:0,vision:{valid:false,pipeline:0,capturedAt:0,targets:[]}};
  let r=await send({type:'start',runId:'replay',seq,time,alliance,cameraFree:true,movementOnly:true,timedReplay:true,singleTilt,shooterPower:1,sensors});
  while(r.running&&time<30.1){
   assert.equal(r.type,'command');assert.match(r.status,/TIMED REPLAY \(NO ODOMETRY\)/);assert(robotAllowed(game.pose));
   for(const k of ['shooterPower','feederPower','intakePower'])assert.equal(r.outputs[k],0);
   if(r.state==='PARK'&&parkAt===null)parkAt=time;if(r.state==='INTAKE')sawPickup=true;
   actuators.apply(r.outputs,.02,time);time=Number((time+.02).toFixed(8));seq++;
   r=await send({type:'step',runId:'replay',seq,time,sensors});
  }
  assert.equal(r.state,'PARK');assert(!r.running&&time<30);assert(sawPickup);
  assert.equal(game.shots,0);assert.equal(game.inventory.length,4);
  assert(Math.hypot(game.pose.x-(alliance===0?-59:59),game.pose.z-(alliance===0?-30:30))<.8,'Recorded commands must reproduce simulated parking from the matching start');
  assert(singleTilt?parkAt<15:parkAt===26);for(const v of r.outputs.wheelPowers)assert.equal(v,0);
  console.log('NO ODOMETRY REPLAY',{alliance,singleTilt,parkAt,finishedAt:time});
 }finally{lines.close();child.stdin.end();child.kill();}
}
