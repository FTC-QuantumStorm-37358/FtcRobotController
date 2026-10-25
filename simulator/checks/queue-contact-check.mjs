import assert from 'node:assert/strict';import * as T from '../dist/vendor/three.module.js';
const ctx=new Proxy({},{get:()=>()=>{},set:()=>true});globalThis.document={createElement:()=>({getContext:()=>ctx})};
const {buildField,buildRobot}=await import('../dist/model.js');const {GamePhysics}=await import('../dist/game-physics.js');const {simulate,makeTarget}=await import('../dist/ballistics.js');const {nextShotSettings}=await import('../dist/next-shot.js');
const setup=()=>{const field=buildField(),robot=buildRobot(),scene=new T.Scene();scene.add(field,robot);const g=new GamePhysics(field,scene,robot);for(const b of [...g.balls]){g.world.removeBody(b.body);b.mesh.removeFromParent();}g.balls=[];g.inventory=[];g.syncInventory();return g;};
const g=setup(),ratios={nectar:.63,pollen:.42},base={power:.4,angle:72,height:6,compression:.15};
for(const type of ['red','blue','pollen'])g.collect(g.createBall(type,[0,2,0]));const target=makeTarget(g.raisedCell());
const preview=()=>{const settings=nextShotSettings(g.nextBall,base,g.pose,ratios);if(!settings)return null;const s=simulate(settings,target);s.ballId=settings.ballId;return s;};
assert.equal(preview().type,'nectar');assert.equal(preview().diameter,3.6);const first=g.nextBall;assert(g.shoot(preview()));assert.equal(g.nextBall.type,'blue');assert.equal(preview().type,'nectar');assert.notEqual(preview().ballId,first.body.id);g.elapsed+=.3;assert(g.shoot(preview()));assert.equal(g.nextBall.type,'pollen');assert.equal(preview().type,'pollen');g.elapsed+=.3;assert(g.shoot(preview()));assert.equal(preview(),null);assert(!g.nextMarker.visible);
const h=setup();h.pose={x:40,z:0,theta:0};h.syncRobot();const ball=h.createBall('pollen',[49.9,1.4,-10]);h.pickup();assert.equal(ball.intakeContact,'blocked');assert.equal(h.inventory.length,0);
// Reproduce side-slide into the front-corner intake zone without first separating.
ball.body.position.x=47*.0254;h.pickup();assert.equal(h.inventory.length,0);ball.body.position.set(40*.0254,ball.r,-20*.0254);h.pickup();assert.equal(ball.intakeContact,null);ball.body.position.z=-10.3*.0254;h.pickup();assert.equal(h.inventory.length,1);
// A physical side impact pushes the ball instead of swallowing it.
const p=setup();p.pose={x:35,z:0,theta:0};p.syncRobot();const pushed=p.createBall('pollen',[45.5,1.4,0]);for(let i=0;i<45;i++){p.drive(0,1,0,1/120);p.step(1/120);}assert.equal(p.inventory.length,0);assert(pushed.body.position.x/.0254>46,'Side collision must push the ball');
// Strafe across a ball just ahead of the front lip: it still must not be collected.
const lip=setup();lip.pose={x:35,z:0,theta:0};lip.syncRobot();lip.createBall('pollen',[46,1.4,-10.5]);for(let i=0;i<60;i++){lip.drive(0,1,0,1/120);lip.step(1/120);}assert.equal(lip.inventory.length,0);
const stale=setup();for(const type of ['red','pollen'])stale.collect(stale.createBall(type,[0,2,0]));const old=nextShotSettings(stale.nextBall,base,stale.pose,ratios),oldShot=simulate(old,makeTarget(stale.raisedCell()));oldShot.ballId=old.ballId;stale.select(1);assert.equal(stale.shoot(oldShot),false);assert.equal(stale.inventory.length,2);
console.log('N → N → P preview timing, empty queue, stale-shot rejection, side-slide exclusion, front intake and physical side pushing passed.');
