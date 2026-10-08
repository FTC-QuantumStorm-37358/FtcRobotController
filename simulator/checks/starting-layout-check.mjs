import assert from 'node:assert/strict';import * as T from '../dist/vendor/three.module.js';
const ctx=new Proxy({},{get:()=>()=>{},set:()=>true});globalThis.document={createElement:()=>({getContext:()=>ctx})};
const {buildField,buildRobot}=await import('../dist/model.js');const {GamePhysics}=await import('../dist/game-physics.js');
const field=buildField(),robot=buildRobot(),scene=new T.Scene();scene.add(field,robot);const g=new GamePhysics(field,scene,robot);
assert.equal(g.balls.length,56);assert.equal(g.inventory.length,4);assert(g.inventory.every(b=>b.type==='pollen'));
for(const type of ['red','blue']){const nectar=g.balls.filter(b=>b.type===type);assert.equal(nectar.filter(b=>b.startZone==='hive').length,3);assert.equal(nectar.filter(b=>b.startZone==='loading').length,5);}
assert.equal(g.balls.filter(b=>b.startZone==='garden').length,20);assert(g.flowers.every(f=>f.queue.length===4));assert(g.hives.every(h=>h.counts.reduce((n,c)=>n+c.nectar,0)===3));
for(let i=0;i<120;i++)g.step(1/60);assert(g.flowers.every(f=>!f.released&&f.queue.length===4));assert(g.hives.every(h=>h.tilts===0&&h.counts.reduce((n,c)=>n+c.nectar,0)===3));
// Empty the inventory without teleporting flowers into it; test release and later movement pickup.
for(const item of g.inventory){item.held=false;scene.add(item.mesh);item.mesh.visible=true;item.body.position.set(0,.1,0);g.world.addBody(item.body);}g.inventory=[];
const flower=g.flowers.find(f=>f.x===-24);g.pose={x:flower.x,z:flower.z+11.4,theta:0};g.syncRobot();const ids=[...flower.queue];g.drive(1,0,0,1/60);assert(flower.released);const distance=g.travelDistance;
for(let i=0;i<120;i++)g.step(1/60);assert.equal(flower.queue.length,0);assert.equal(g.inventory.length,0,'Touch alone must not collect pollen');assert(ids.every(b=>!b.storedInFlower&&b.body.world===g.world));assert(g.flowers.filter(f=>f!==flower).every(f=>f.queue.length===4));assert.equal(g.travelDistance,distance);
// Move over an actual released ball after the required post-release movement.
const item=ids[0];g.pose={x:40,z:0,theta:0};g.pickup();g.pose={x:item.body.position.x/.0254,z:item.body.position.z/.0254+10.2,theta:0};g.travelDistance+=3;g.syncRobot();g.pickup();assert(g.inventory.includes(item));assert(g.inventory.length<=4);
g.reset();assert.equal(g.inventory.length,4);assert(g.flowers.every(f=>f.queue.length===4&&!f.released));assert.equal(g.balls.length,56);
console.log('Correct 40-pollen/16-nectar setup; stable hive preloads; staged flower release; movement-only pickup; reset passed.');
