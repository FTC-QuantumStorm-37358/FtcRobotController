import {SimSensors,toBrowserPose} from './sim-sensors.js';
import {SimActuators} from './sim-actuators.js';

const tick=.02;
/** Transport and simulated actuators only. Autonomous decisions run exclusively in Java. */
export class JavaAutonomousClient {
 constructor(game,{shoot=()=>false,wake=()=>{},onStatus=()=>{},socketFactory=url=>new WebSocket(url),url=null,visionOptions={}}={}){
  Object.assign(this,{game,shoot,wake,onStatus,socketFactory,url});this.sensors=new SimSensors(game,visionOptions);this.actuators=new SimActuators(game,shoot);
  this.running=false;this.available=false;this.status='Checking Java controller…';this.state='STOPPED';this.generation=0;this.time=0;this.navigation={destination:null,route:[]};
 }
 async checkServer(){try{const r=await fetch('/api/java-status');const result=await r.json();this.available=result.ready;this.status=result.message;}catch{this.status='Java autonomous requires the local npm start server.';}this.onStatus();return this.available;}
 start(alliance=0,shooterPower=.395){
  this.stop('Ready');this.running=true;this.state='CONNECTING';this.status='Connecting to Java…';
  this.time=0;this.accumulator=0;this.seq=0;this.pipeline=0;this.queued=null;this.actuators.reset();this.sensors.reset();
  const generation=++this.generation;this.runId=globalThis.crypto.randomUUID();
  try{this.socket=this.socketFactory(this.url??`${location.protocol==='https:'?'wss':'ws'}://${location.host}/robot`);}catch(e){this.stop(e.message);return;}
  const socket=this.socket,current=()=>generation===this.generation&&this.running;
  this.timer=setTimeout(()=>{if(current())this.stop('Java connection timed out');},6000);
  socket.onmessage=event=>{
   if(!current())return;
   try{
    const m=JSON.parse(event.data);
    if(m.type==='error'){this.stop(m.message??'Java error');return;}
    if(m.type==='ready'){
     if(m.protocol!==1||m.engine!=='java'||m.tickSeconds!==tick)throw new Error('Java protocol mismatch');
     this.tagMap={nearTags:m.nearTags,farTags:m.farTags};
     this.send({type:'start',alliance:Number(alliance),shooterPower:Number(shooterPower),sensors:this.sensors.read(0,0,this.tagMap)});return;
    }
    if(m.type!=='command'||m.protocol!==1||m.runId!==this.runId||m.seq!==this.seq||Math.abs(m.time-this.time)>1e-8)throw new Error('Stale Java command');
    clearTimeout(this.timer);this.pending=false;
    const o=m.outputs;if(!o||!['forward','strafeRight','turnClockwise','shooterPower','feederPower','intakePower'].every(k=>Number.isFinite(o[k])&&Math.abs(o[k])<=1)||![0,1].includes(o.pipeline)||!Array.isArray(o.wheelPowers)||o.wheelPowers.length!==4||!o.wheelPowers.every(p=>Number.isFinite(p)&&Math.abs(p)<=1))throw new Error('Invalid actuator command');
    this.state=m.state;this.status=m.status;
    const nav=m.navigation,validPose=p=>p&&[p.x,p.y,p.heading].every(Number.isFinite);
    if(nav&&Array.isArray(nav.route)&&nav.route.every(validPose)&&(nav.destination===null||validPose(nav.destination)))this.navigation={destination:nav.destination?toBrowserPose(nav.destination):null,route:nav.route.map(toBrowserPose)};
    if(!m.running){this.running=false;this.queued=null;this.game.intakeEnabled=true;socket.close();}
    else this.queued=o;
    this.onStatus();this.wake();
   }catch(e){this.stop(e.message);}
  };
  socket.onerror=()=>{if(current())this.stop('Java connection failed — start the local server.');};
  socket.onclose=()=>{if(current())this.stop('Java disconnected; autonomous stopped.');};this.onStatus();this.wake();
 }
 send(message){
  this.pending=true;clearTimeout(this.timer);this.timer=setTimeout(()=>this.stop('Java response timed out; autonomous stopped.'),2500);
  this.socket.send(JSON.stringify({protocol:1,runId:this.runId,seq:this.seq,time:this.time,...message}));
 }
 stop(reason='Stopped — manual control'){
  this.generation++;this.running=false;this.state='STOPPED';this.navigation={destination:null,route:[]};this.queued=null;this.pending=false;clearTimeout(this.timer);
  if(this.socket){this.socket.close();this.socket=null;}this.game.intakeEnabled=true;this.game.intakeMotion={moving:false,advancing:false};this.status=reason;this.onStatus();
 }
 /** Advance the physical world once per matching Java response; freeze it while awaiting Java. */
 update(dt){
  if(!this.running)return {advanced:false,moved:false};this.accumulator=Math.min(.2,this.accumulator+Math.max(0,dt));
  if(!this.queued||this.accumulator+1e-9<tick)return {advanced:false,moved:false};
  const o=this.queued;this.queued=null;this.accumulator-=tick;this.pipeline=o.pipeline;const moved=this.actuators.apply(o,tick,this.time);
  this.time=Number((this.time+tick).toFixed(8));this.seq++;
  this.send({type:'step',sensors:this.sensors.read(this.time,this.pipeline,this.tagMap)});
  return {advanced:true,moved};
 }
}
