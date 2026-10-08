import * as T from './vendor/three.module.js';
import {INCH} from './ballistics.js';
import {obstacles} from './motion.js';

export const toJavaPose=p=>({x:p.x,y:-p.z,heading:Math.atan2(Math.sin(p.theta+Math.PI/2),Math.cos(p.theta+Math.PI/2))});
export const toBrowserPose=p=>({x:p.x,z:-p.y,theta:p.heading-Math.PI/2});
const degrees=180/Math.PI;
function boxBlocks(a,b,box){
 let low=0,high=1;
 for(const axis of ['x','y','z']){const d=b[axis]-a[axis],min=box[axis][0],max=box[axis][1];if(Math.abs(d)<1e-10){if(a[axis]<min||a[axis]>max)return false;}else{const u=(min-a[axis])/d,v=(max-a[axis])/d;low=Math.max(low,Math.min(u,v));high=Math.min(high,Math.max(u,v));if(low>high)return false;}}
 return high>0&&low<1;
}
/** Synthetic camera measurements, not actual image recognition. */
export class SimSensors {
 constructor(game,{latency=.06,fps=30,horizontalFov=62,verticalFov=70,range=180,enabled=true,dropout=0,noiseDegrees=0}={}){
  Object.assign(this,{game,latency,fps,horizontalFov,verticalFov,range,enabled,dropout,noiseDegrees});this.reset();
 }
 reset(){this.pipeline=0;this.nextCapture=0;this.pending=[];this.latest={valid:false,pipeline:0,capturedAt:0,targets:[]};this.randomState=12345;this.frame=0;}
 random(){this.randomState=(1664525*this.randomState+1013904223)>>>0;return this.randomState/4294967296;}
 measure(point,time,id,normal=null){
  const p=this.game.pose,forward=new T.Vector3(-Math.sin(p.theta),0,-Math.cos(p.theta)),right=new T.Vector3(Math.cos(p.theta),0,-Math.sin(p.theta));
  const origin=new T.Vector3(p.x,8,p.z).addScaledVector(forward,2),delta=point.clone().sub(origin),distance=delta.length();
  if(distance>this.range||distance<1||normal&&normal.dot(origin.clone().sub(point).normalize())<.1)return null;
  const front=delta.dot(forward),lateral=delta.dot(right);if(front<=0)return null;
  const tx=Math.atan2(lateral,front)*degrees,ty=Math.atan2(delta.y,Math.hypot(front,lateral))*degrees-(this.pipeline===0?35:-25);
  if(Math.abs(tx)>this.horizontalFov/2||Math.abs(ty)>this.verticalFov/2)return null;
  if(obstacles.some((b,i)=>boxBlocks(origin,point,{x:[b.x-b.hx,b.x+b.hx],y:[0,i<4?43.95:21.6],z:[b.z-b.hz,b.z+b.hz]})))return null;
  const noise=()=>this.noiseDegrees?(this.random()-.5)*2*this.noiseDegrees:0;
  return {id,tx:tx+noise(),ty:ty+noise(),distance};
 }
 capture(time,tagMap){
  const targets=[];this.game.field.updateMatrixWorld(true);
  if(this.enabled&&this.random()>=this.dropout){
   if(this.pipeline===0){for(const hive of this.game.hives)for(let face=0;face<hive.cells.length;face++){
    const cell=hive.cells[face],normal=new T.Vector3(0,-1,0).transformDirection(cell.matrixWorld),ids=(face===0?tagMap.nearTags:tagMap.farTags)[hive.index];
    for(let i=0;i<ids.length;i++){const point=new T.Vector3(i%2?2:-2,-.15,i<2?-2:2).applyMatrix4(cell.matrixWorld);const result=this.measure(point,time,ids[i],normal);if(result)targets.push({...result,heightInches:point.y});}
   }}else for(const ball of this.game.balls){
    if(ball.held||ball.storedInFlower||ball.body.position.y>ball.r+.025)continue;
    const point=new T.Vector3(ball.body.position.x/INCH,ball.body.position.y/INCH,ball.body.position.z/INCH),result=this.measure(point,time,-1);if(result)targets.push(result);
   }
  }
  targets.sort((a,b)=>a.distance-b.distance);const packet={valid:targets.length>0,pipeline:this.pipeline,capturedAt:time,targets:targets.map(({id,tx,ty,heightInches})=>({id,tx,ty,...(heightInches===undefined?{}:{heightInches})}))};
  this.pending.push({releaseAt:time+this.latency,packet});this.frame++;
 }
 read(time,pipeline,tagMap){
  if(pipeline!==this.pipeline){this.pipeline=pipeline;this.pending=[];this.latest={valid:false,pipeline,capturedAt:time,targets:[]};this.nextCapture=time+.08;}
  if(time+1e-9>=this.nextCapture){this.capture(time,tagMap);this.nextCapture=time+1/this.fps;}
  while(this.pending.length&&this.pending[0].releaseAt<=time+1e-9)this.latest=this.pending.shift().packet;
  const vision=this.enabled?this.latest:{valid:false,pipeline,capturedAt:time,targets:[]};
  return {pose:toJavaPose(this.game.pose),sampledAt:time,odometryValid:true,cameraConnected:true,ballCount:this.game.inventory.length,vision};
 }
}
