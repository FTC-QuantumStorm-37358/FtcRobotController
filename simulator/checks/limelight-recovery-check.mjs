import assert from 'node:assert/strict';
import {createInterface} from 'node:readline';
import {spawnSync} from 'node:child_process';
import path from 'node:path';
import * as T from '../dist/vendor/three.module.js';
import {compileJava,launchJava,java,classes} from '../bridge/java-runtime.mjs';
import {SimSensors} from '../dist/sim-sensors.js';
import {SimActuators} from '../dist/sim-actuators.js';
import {fireNextBall} from '../dist/next-shot.js';
import {simulate,makeTarget,fencePose} from '../dist/ballistics.js';
const ctx=new Proxy({},{get:()=>()=>{},set:()=>true});globalThis.document={createElement:()=>({getContext:()=>ctx})};
const {buildField,buildRobot}=await import('../dist/model.js');
const {GamePhysics,robotAllowed}=await import('../dist/game-physics.js');
await compileJava([path.resolve('robot/tests/RecoveryChecks.java')]);
const safety=spawnSync(java,['-cp',classes,'org.firstinspires.ftc.teamcode.biobuzz.RecoveryChecks'],{encoding:'utf8'});
assert.equal(safety.status,0,safety.stdout+safety.stderr);console.log(safety.stdout.trim());
for(const alliance of [0,1]){
 const child=launchJava(),lines=createInterface({input:child.stdout}),queue=[],waiters=[];
 lines.on('line',line=>{const packet=JSON.parse(line);if(waiters.length)waiters.shift()(packet);else queue.push(packet);});
 const next=()=>queue.length?Promise.resolve(queue.shift()):new Promise(resolve=>waiters.push(resolve));
 const ready=await next(),send=async packet=>{child.stdin.write(JSON.stringify({protocol:1,...packet})+'\n');return next();};
 try{
  const field=buildField(),robot=buildRobot(),scene=new T.Scene();scene.add(field,robot);
  const game=new GamePhysics(field,scene,robot);game.pose=fencePose(alliance);game.syncRobot();
  const sensors=new SimSensors(game);sensors.ballColorFilter=['pollen',alliance===0?'red':'blue'];
  let time=0,seq=0,last='',sawRecovery=false,parkAt=null,sawTilt=false;
  const fired=[];
  const actuators=new SimActuators(game,power=>{const source=game.nextBall;const ok=fireNextBall(game,{power,angle:66.5,height:6,compression:.15},makeTarget(game.raisedCell(alliance)),{pollen:.5,nectar:.5},simulate);if(ok)fired.push(source);return ok;});
  const read=pipeline=>sensors.read(time,pipeline,ready);
  let packet=await send({type:'start',runId:'recovery',seq,time,alliance,visionRecovery:true,shooterPower:.395,sensors:read(0)});
  while(packet.running&&time<30.1){
   assert.equal(packet.type,'command',packet.message);assert(robotAllowed(game.pose));
   assert(!['ERROR','STOPPED'].includes(packet.state),packet.status);
   if(packet.state!==last){console.log(alliance,time,packet.state,packet.status,game.pose);last=packet.state;}
   sawTilt ||= packet.status.includes('Hive tilt confirmed');
   if(['CLEAR_FENCE','SEARCH_POLLEN','APPROACH_POLLEN'].includes(packet.state)){
    assert(sawTilt&&game.hives[alliance].tilts>=1,'Recovery requires an independently completed physical Hive tilt');
    assert.equal(game.shots,4,'Only four preloads may fire');
   }
   if(packet.state==='APPROACH_POLLEN'){sawRecovery=true;assert.equal(packet.outputs.shooterPower,0);assert.equal(packet.outputs.feederPower,0);}
   if(packet.state==='PARK'&&parkAt===null)parkAt=time;
   actuators.apply(packet.outputs,.02,time);
   time=Number((time+.02).toFixed(8));seq++;
   packet=await send({type:'step',runId:'recovery',seq,time,sensors:read(packet.outputs.pipeline)});
  }
  console.log('RECOVERY RESULT',{alliance,time,shots:game.shots,parkAt,sawTilt,sawRecovery,inventory:game.inventory.map(b=>({type:b.type,source:b.startZone,spills:b.spills})),status:packet.status,pose:game.pose});
  assert.equal(fired.length,4);assert(fired.every(b=>b.startZone==='robot'));
  assert(sawTilt&&sawRecovery,'Must see tilt and a camera-directed approach');
  assert(game.inventory.some(b=>b.type==='pollen'&&b.spills>0),'Must physically collect pollen spilled from the Hive, not Garden/Flower stock');
  assert(game.inventory.some(b=>b.type===(alliance===0?'red':'blue')&&b.spills>0),'Must physically collect alliance nectar spilled from the Hive');
  assert(game.inventory.length>1&&game.inventory.length<=4,'Collect multiple real balls without exceeding capacity');
  assert(parkAt<=26,'Park after recovery, or earlier if the return budget requires it');assert(time<30&&!packet.running,'Finish parking before hard deadline');
  for(const key of ['forward','strafeRight','turnClockwise','shooterPower','feederPower','intakePower'])assert.equal(packet.outputs[key],0);
 }finally{lines.close();child.stdin.end();child.kill();}
}
