import assert from 'node:assert/strict';import * as T from '../dist/vendor/three.module.js';
const ctx=new Proxy({},{get:()=>()=>{},set:()=>true});globalThis.document={createElement:()=>({getContext:()=>ctx})};
const {buildField,buildRobot}=await import('../dist/model.js');const {GamePhysics}=await import('../dist/game-physics.js');
const field=buildField(),robot=buildRobot(),scene=new T.Scene();scene.add(field,robot);const g=new GamePhysics(field,scene,robot);
assert.equal(g.balls.length,44);assert.equal(g.inventory.length,4);assert(g.inventory.every(b=>b.type==='pollen'));
const staged=[];
for(const type of ['red','blue']){
 const nectar=g.balls.filter(b=>b.type===type);assert.equal(nectar.filter(b=>b.startZone==='hive').length,3);
 const stock=nectar.filter(b=>b.startZone==='alliance');assert.equal(stock.length,5);
 for(const b of stock){assert(b.storedOutside&&!b.body.world);assert.equal(Math.sign(b.body.position.x),type==='red'?-1:1);assert(Math.abs(b.body.position.x)/.0254>73+b.r/.0254);staged.push([b,b.body.position.clone()]);}
}
assert(!g.balls.some(b=>b.startZone==='loading'),'No nectar in the Loading Zones');
assert.equal(g.balls.filter(b=>b.startZone==='garden').length,8);assert(g.flowers.every(f=>f.queue.length===4));assert(g.hives.every(h=>h.counts.reduce((n,c)=>n+c.nectar,0)===3));
for(let i=0;i<120;i++)g.step(1/60);assert(g.flowers.every(f=>!f.released&&f.queue.length===4));assert(g.hives.every(h=>h.tilts===0&&h.counts.reduce((n,c)=>n+c.nectar,0)===3));
for(const [b,p] of staged){assert.deepEqual(b.body.position,p,'Outside stock must not be clamped into the field');assert(b.mesh.visible);}
const loaded=g.inventory;g.inventory=[];for(const [b] of staged)assert.equal(g.collect(b),false,'Outside stock cannot be collected');g.inventory=loaded;
// Empty the inventory without teleporting flowers into it; test release and later movement pickup.
for(const item of g.inventory){item.held=false;scene.add(item.mesh);item.mesh.visible=true;item.body.position.set(0,.1,0);g.world.addBody(item.body);}g.inventory=[];
const flower=g.flowers.find(f=>f.x===-24);g.pose={x:flower.x,z:flower.z+11.4,theta:0};g.syncRobot();const ids=[...flower.queue];g.drive(1,0,0,1/60);assert(flower.released);const distance=g.travelDistance;
for(let i=0;i<120;i++)g.step(1/60);assert.equal(flower.queue.length,0);assert.equal(g.inventory.length,0,'Touch alone must not collect pollen');assert(ids.every(b=>!b.storedInFlower&&b.body.world===g.world));assert(g.flowers.filter(f=>f!==flower).every(f=>f.queue.length===4));assert.equal(g.travelDistance,distance);
// Move over an actual released ball after the required post-release movement.
const item=ids[0];g.pose={x:40,z:0,theta:0};g.pickup();g.pose={x:item.body.position.x/.0254,z:item.body.position.z/.0254+10.2,theta:0};g.travelDistance+=3;g.syncRobot();g.pickup();assert(g.inventory.includes(item));assert(g.inventory.length<=4);
g.reset();assert.equal(g.inventory.length,4);assert(g.flowers.every(f=>f.queue.length===4&&!f.released));assert.equal(g.balls.length,44);
console.log('Requested 28-pollen/16-nectar setup (4 per Garden); stable hive preloads; staged flower release; movement-only pickup; reset passed.');
