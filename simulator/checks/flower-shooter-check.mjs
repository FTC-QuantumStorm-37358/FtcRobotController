import assert from 'node:assert/strict';
import * as T from '../dist/vendor/three.module.js';
const ctx=new Proxy({},{get:()=>()=>{},set:()=>true});globalThis.document={createElement:()=>({getContext:()=>ctx})};
const {buildField,buildRobot}=await import('../dist/model.js');
const {GamePhysics}=await import('../dist/game-physics.js');
const {createShooterMount,updateShooterMount}=await import('../dist/shooter-model.js');
const {launcher}=await import('../dist/ballistics.js');
const setup=()=>{const f=buildField(),r=buildRobot(),s=new T.Scene();s.add(f,r);return new GamePhysics(f,s,r);};
for(let index=0;index<4;index++){
 const g=setup(),f=g.flowers[index],released=[...f.queue];g.pose={x:f.x+f.direction.x*20,z:f.z+f.direction.z*20,theta:Math.atan2(f.direction.x,f.direction.z)};g.syncRobot();
 for(let k=0;k<110;k++){g.drive(1,0,0,1/120);g.step(1/120);}assert(f.released);
 for(let k=0;k<360;k++)g.step(1/120);assert.equal(f.queue.length,0);assert.equal(g.inventory.length,4);
 const sideDisplacements=[];
 for(const b of released){assert(b.flowerDeflected);assert(!b.held);assert.equal(b.body.world,g.world);assert(b.mesh.visible&&b.mesh.parent===g.scene);assert(b.body.position.y>=b.r-.005);
  const dx=b.body.position.x/.0254-g.pose.x,dz=b.body.position.z/.0254-g.pose.z,c=Math.cos(g.pose.theta),s=Math.sin(g.pose.theta),lx=c*dx-s*dz,lz=s*dx+c*dz;
  assert(Math.hypot(Math.max(0,Math.abs(lx)-8.75),Math.max(0,-9.05-lz,lz-8.055))>=b.r/.0254-.05,'Rejected pollen must stay outside full robot');
  sideDisplacements.push((b.body.position.x/.0254-f.x)*(-f.direction.z)+(b.body.position.z/.0254-f.z)*f.direction.x);
 }
 assert.equal(sideDisplacements.filter(x=>x<0).length,2);assert.equal(sideDisplacements.filter(x=>x>0).length,2);
 assert.equal(g.balls.length,56,'No released balls may disappear from world stock');
}
// Move and rotate for 90 rendered frames WITHOUT rebuilding the shooter or preview.
const g=setup();g.pose={x:40,z:35,theta:0};g.syncRobot();const mount=createShooterMount(g.robot),settings={angle:71.5,height:6,compression:.15};updateShooterMount(mount,settings,2.8);const wheel=mount.children[0],geometry=wheel.geometry;
for(let i=0;i<90;i++){g.drive(i<40?1:0,i>=40?1:0,.3,1/60);g.robot.updateMatrixWorld(true);const expected=launcher(settings.angle,settings.height,settings.compression,g.pose,2.8).center;assert(wheel.getWorldPosition(new T.Vector3()).distanceTo(expected)<1e-9);assert.equal(mount.parent,g.robot);}
updateShooterMount(mount,settings,2.8);assert.equal(wheel.geometry,geometry);assert.equal(mount.children[0],wheel,'Unchanged guide must not rebuild during preview refresh');
updateShooterMount(mount,{...settings,angle:75},3.6);g.robot.updateMatrixWorld(true);assert(mount.children[0].getWorldPosition(new T.Vector3()).distanceTo(launcher(75,6,.15,g.pose,3.6).center)<1e-9);
console.log('All four full-intake flower releases remain visible/world-active and split 2 left/2 right; fixed shooter follows 90 movement frames without rebuilding passed.');
