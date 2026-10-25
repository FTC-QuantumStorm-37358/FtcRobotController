import assert from 'node:assert/strict';
import {createInterface} from 'node:readline';
import * as T from '../dist/vendor/three.module.js';
import {compileJava,launchJava} from '../bridge/java-runtime.mjs';
import {SimSensors} from '../dist/sim-sensors.js';
import {SimActuators} from '../dist/sim-actuators.js';
import {fencePose} from '../dist/ballistics.js';
const ctx=new Proxy({},{get:()=>()=>{},set:()=>true});globalThis.document={createElement:()=>({getContext:()=>ctx})};
const {buildField,buildRobot}=await import('../dist/model.js');
const {GamePhysics,robotAllowed}=await import('../dist/game-physics.js');
await compileJava();
for(const alliance of [0,1])for(const reportedCount of [-1,4]){
 const child=launchJava(),lines=createInterface({input:child.stdout}),queue=[],waiters=[];
 lines.on('line',line=>{const m=JSON.parse(line);if(waiters.length)waiters.shift()(m);else queue.push(m);});
 const next=()=>queue.length?Promise.resolve(queue.shift()):new Promise(resolve=>waiters.push(resolve));
 const ready=await next(),send=async m=>{child.stdin.write(JSON.stringify({protocol:1,...m})+'\n');return next();};
 try{
  const field=buildField(),robot=buildRobot(),scene=new T.Scene();scene.add(field,robot);
  const game=new GamePhysics(field,scene,robot);game.pose=fencePose(alliance);game.syncRobot();
  const sensors=new SimSensors(game,{enabled:false}),actuators=new SimActuators(game,()=>assert.fail('Movement-only must never fire'));
  let time=0,seq=0,parkAt=null,last='',gardenAt=null,flowerAt=null;const pauses=[],visits=[];
  const read=()=>{const s=sensors.read(time,0,ready);s.cameraConnected=false;s.ballCount=reportedCount;return s;};
  let r=await send({type:'start',runId:'dry',seq,time,alliance,cameraFree:true,movementOnly:true,shooterPower:1,sensors:read()});
  while(r.running&&time<30.1){
   assert.equal(r.type,'command');assert.notEqual(r.state,'ERROR',r.status);assert(robotAllowed(game.pose));
   for(const key of ['shooterPower','feederPower','intakePower','pipeline'])assert.equal(r.outputs[key],0,`${key} must stay OFF in every state`);
   const name=r.status.split(' · ')[0];if(name!==last){visits.push(name);if(r.state==='MOVING'&&name.includes('Garden'))assert(Math.abs(time-3)<.1,'Opening three-second pause is retained');last=name;}
   if(r.state==='INTAKE'){
    if(r.status.includes('INTAKE · GARDEN')&&gardenAt===null)gardenAt=time;
    if(r.status.includes('INTAKE · FLOWER')&&flowerAt===null)flowerAt=time;
   }else{
    if(gardenAt!==null){pauses.push(['garden',time-gardenAt]);gardenAt=null;}
    if(flowerAt!==null){pauses.push(['flower',time-flowerAt]);flowerAt=null;}
   }
   if(r.state==='PARK'&&parkAt===null)parkAt=time;
   actuators.apply(r.outputs,.02,time);assert.equal(game.shots,0);assert.equal(game.inventory.length,4,'No fabricated inventory or real mechanism actions');
   time=Number((time+.02).toFixed(8));seq++;r=await send({type:'step',runId:'dry',seq,time,sensors:read()});
  }
  assert.equal(r.state,'PARK');assert(!r.running);assert(time<30,r.status);
  assert(pauses.some(([name,d])=>name==='garden'&&Math.abs(d-.14)<.03),'Simulator Garden dwell retained even with full/unknown count');
  assert.equal(parkAt,26);assert(pauses.some(([name,d])=>name==='flower'&&Math.abs(d-2.34)<.03));
  // The rear-facing start needs fence-exit turn clearance. Optional leftovers
  // may be skipped when that travel consumes the spare-time budget.
  assert(Math.hypot(game.pose.x-(alliance===0?-59:59),game.pose.z-(alliance===0?-30:30))<.7);
  for(const key of ['forward','strafeRight','turnClockwise','shooterPower','feederPower','intakePower'])assert.equal(r.outputs[key],0);
  console.log('DRIVE ONLY',{alliance,reportedCount,parkAt,finishedAt:time,pauses});
 }finally{lines.close();child.stdin.end();child.kill();}
}
