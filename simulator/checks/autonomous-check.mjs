import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {createInterface} from 'node:readline';
import * as T from '../dist/vendor/three.module.js';
import {spawnSync} from 'node:child_process';
import path from 'node:path';
import {compileJava,launchJava,java,classes} from '../bridge/java-runtime.mjs';
import {SimSensors,toJavaPose,toBrowserPose} from '../dist/sim-sensors.js';
import {SimActuators} from '../dist/sim-actuators.js';
import {JavaAutonomousClient} from '../dist/autonomous.js';
const ctx=new Proxy({},{get:()=>()=>{},set:()=>true});globalThis.document={createElement:()=>({getContext:()=>ctx})};
const {buildField,buildRobot}=await import('../dist/model.js');
const {GamePhysics,robotAllowed}=await import('../dist/game-physics.js');
const {fencePose,simulate,makeTarget}=await import('../dist/ballistics.js');
const {fireNextBall}=await import('../dist/next-shot.js');
function setup(){const f=buildField(),r=buildRobot(),s=new T.Scene();s.add(f,r);return new GamePhysics(f,s,r);}
await compileJava([path.resolve('robot/tests/ControllerChecks.java'),path.resolve('robot/tests/HiveVisionChecks.java')]);
const hiveUnit=spawnSync(java,['-cp',classes,'org.firstinspires.ftc.teamcode.biobuzz.HiveVisionChecks'],{encoding:'utf8'});assert.equal(hiveUnit.status,0,hiveUnit.stdout+hiveUnit.stderr);console.log(hiveUnit.stdout.trim());
const unit=spawnSync(java,['-cp',classes,'org.firstinspires.ftc.teamcode.biobuzz.ControllerChecks'],{encoding:'utf8'});assert.equal(unit.status,0,unit.stdout+unit.stderr);console.log(unit.stdout.trim());
async function runner(){
 const child=launchJava(),lines=createInterface({input:child.stdout}),queue=[],waiters=[];let ready;
 lines.on('line',line=>{const m=JSON.parse(line);if(waiters.length)waiters.shift()(m);else queue.push(m);});
 const next=()=>queue.length?Promise.resolve(queue.shift()):new Promise(resolve=>waiters.push(resolve));
 ready=await next();assert.equal(ready.engine,'java');
 return {ready,async send(m){child.stdin.write(JSON.stringify({protocol:1,...m})+'\n');return next();},close(){lines.close();child.stdin.end();child.kill();}};
}
for(const alliance of [0,1]){
 const r=await runner();try{
  const game=setup();game.pose=fencePose(alliance);game.syncRobot();const camera=new SimSensors(game);let time=0,seq=0,shots=0;const states=new Set();const actuators=new SimActuators(game,power=>{const fired=fireNextBall(game,{power,angle:66.5,height:6,compression:.15},makeTarget(game.raisedCell(alliance)),{pollen:.5,nectar:.5},simulate);if(fired)shots++;return fired;});
  let response=await r.send({type:'start',runId:'physical'+alliance,seq,time,alliance,shooterPower:.395,sensors:camera.read(time,0,r.ready)});
  while(response.running&&time<30.1){
   assert.equal(response.type,'command');states.add(response.state);const o=response.outputs;actuators.apply(o,.02,time);assert(robotAllowed(game.pose));assert(game.inventory.length<=4);
   time=Number((time+.02).toFixed(8));seq++;
   response=await r.send({type:'step',runId:'physical'+alliance,seq,time,sensors:camera.read(time,o.pipeline,r.ready)});
  }
  console.log({alliance,time,shots,inventory:game.inventory.length,states:[...states],status:response.status});
  assert.equal(response.type,'command');assert.equal(response.running,false);assert.equal(response.state,'PARK');assert(shots>=4);assert(states.has('INTAKE'));assert(time<30);
  const park=alliance===0?{x:-59,z:-36}:{x:59,z:36};
  assert(Math.hypot(game.pose.x-park.x,game.pose.z-park.z)<.7,'Must actually reach parking, not merely enter PARK');
  for(const key of ['forward','strafeRight','turnClockwise','shooterPower','feederPower','intakePower'])assert.equal(response.outputs[key],0,'Parked actuators must stop');
 }finally{r.close();}
}
// No tag means no feed, even when robot pose is perfect and the queue is full.
const r=await runner();try{
 const game=setup(),camera=new SimSensors(game,{enabled:false});let time=0,seq=0;
 let response=await r.send({type:'start',runId:'missing-tag',seq,time,alliance:0,shooterPower:.395,sensors:camera.read(time,0,r.ready)});
 for(let i=0;i<200;i++){assert.equal(response.outputs.feederPower,0);time=Number((time+.02).toFixed(8));seq++;response=await r.send({type:'step',runId:'missing-tag',seq,time,sensors:camera.read(time,0,r.ready)});}
 assert.match(response.status,/AprilTag not found/);
 // Reject out-of-order replay and stop rather than keep the old motor command.
 const error=await r.send({type:'step',runId:'missing-tag',seq,time,sensors:camera.read(time,0,r.ready)});assert.equal(error.type,'error');assert.match(error.message,/out-of-order/);
}finally{r.close();}
// Sensor timestamps and pipelines must remain physically delayed; headings have explicit signs/units.
const game=setup(),camera=new SimSensors(game),map={nearTags:[[30,31,32,33],[38,39,40,41]],farTags:[[34,35,36,37],[42,43,44,45]]};
assert.equal(camera.read(0,0,map).vision.valid,false);assert(camera.read(.08,0,map).vision.targets.some(t=>t.id>=0));assert.equal(camera.read(.1,1,map).vision.valid,false);
for(const theta of [-Math.PI,0,.7,Math.PI]){const p={x:12,z:-45,theta},q=toBrowserPose(toJavaPose(p));assert(Math.abs(q.x-p.x)+Math.abs(q.z-p.z)<1e-8);assert(Math.abs(Math.atan2(Math.sin(q.theta-theta),Math.cos(q.theta-theta)))<1e-8);}
const driver=new JavaAutonomousClient(game);driver.running=true;driver.pending=true;const before={...game.pose},elapsed=game.elapsed;driver.update(.2);assert.deepEqual(game.pose,before);assert.equal(game.elapsed,elapsed);driver.stop();assert.equal(driver.running,false);assert.deepEqual(driver.update(.1),{advanced:false,moved:false});
const html=readFileSync('dist/index.html','utf8'),js=readFileSync('dist/shot.js','utf8');assert.equal(html,readFileSync('dist/shot.html','utf8'));assert.match(js,/JavaAutonomousClient/);assert.match(js,/active=auto.running\|\|/);assert.match(js,/e.code==='Escape'/);assert.match(js,/Paused — page hidden/);assert.doesNotMatch(readFileSync('dist/autonomous.js','utf8'),/class AutonomousDriver|planPath|case 'SHOOTING'/);
for(const id of ['autonomous','autonomous-status','autonomous-pose','autonomous-goal','autonomous-progress'])assert(html.includes(`id="${id}"`));assert.match(js,/auto\.navigation\.route/);
console.log('Actual JVM autonomy, physical firing/refill, missing-tag guard, replay rejection, delayed vision and manual stop passed.');
