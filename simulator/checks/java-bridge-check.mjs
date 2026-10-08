import assert from 'node:assert/strict';
import {spawn} from 'node:child_process';
import {readFileSync} from 'node:fs';
import {connect} from 'node:net';
import * as T from '../dist/vendor/three.module.js';
const ctx=new Proxy({},{get:()=>()=>{},set:()=>true});globalThis.document={createElement:()=>({getContext:()=>ctx})};
const {buildField,buildRobot}=await import('../dist/model.js');
const {GamePhysics}=await import('../dist/game-physics.js');
const {JavaAutonomousClient}=await import('../dist/autonomous.js');
const {simulate,makeTarget}=await import('../dist/ballistics.js');
const {fireNextBall}=await import('../dist/next-shot.js');
const child=spawn(process.execPath,['server.mjs'],{env:{...process.env,PORT:'0'},stdio:['ignore','pipe','pipe']});
let logs='';
const port=await new Promise((resolve,reject)=>{
 const timeout=setTimeout(()=>reject(new Error('Server startup timed out: '+logs)),10000);
 child.stdout.on('data',d=>{logs+=d;const m=logs.match(/localhost:(\d+)/);if(m){clearTimeout(timeout);resolve(Number(m[1]));}});
 child.stderr.on('data',d=>logs+=d);child.on('error',reject);child.on('exit',code=>{if(!logs.match(/localhost:/))reject(new Error('Server failed '+code+': '+logs));});
});
try{
 const status=await (await fetch(`http://localhost:${port}/api/java-status`)).json();assert.equal(status.ready,true,status.message);
 const html=await (await fetch(`http://localhost:${port}/`)).text();assert.match(html,/id="autonomous"/);assert.equal(html,readFileSync('dist/index.html','utf8'));
 const f=buildField(),robot=buildRobot(),scene=new T.Scene();scene.add(f,robot);const game=new GamePhysics(f,scene,robot);let shots=0,resolveWake;
 const driver=new JavaAutonomousClient(game,{url:`ws://localhost:${port}/robot`,wake:()=>resolveWake?.(),shoot:power=>{const fired=fireNextBall(game,{power,angle:66.5,height:6,compression:.15},makeTarget(game.raisedCell(0)),{pollen:.5,nectar:.5},simulate);if(fired)shots++;return fired;}});
 driver.start(0,.395);
 async function waitCommand(){const timeout=setTimeout(()=>resolveWake?.(),3000);while(driver.running&&!driver.queued)await new Promise(resolve=>resolveWake=resolve);clearTimeout(timeout);assert(driver.running,driver.status);}
 for(let i=0;i<175;i++){await waitCommand();driver.update(.02);}
 assert.equal(shots,4,'WebSocket client must apply real Java feed outputs');assert.equal(game.inventory.length,0);assert.equal(driver.time,3.5);assert(Array.isArray(driver.navigation.route));
 const before={...game.pose};driver.stop();assert.equal(driver.running,false);driver.update(10);assert.deepEqual(game.pose,before);
 await new Promise(resolve=>setTimeout(resolve,50));
 // Reconnect starts a fresh JVM with a fresh sequence; closing it cancels old outputs.
 driver.start(1,.3);await waitCommand();assert.equal(driver.seq,0);assert.equal(driver.time,0);driver.stop();
 const denied=await new Promise((resolve,reject)=>{const socket=connect(port,'127.0.0.1',()=>socket.write(`GET /robot HTTP/1.1\r\nHost: localhost:${port}\r\nOrigin: http://unrelated.example\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Version: 13\r\nSec-WebSocket-Key: MDEyMzQ1Njc4OWFiY2RlZg==\r\n\r\n`));let text='';socket.on('data',d=>text+=d);socket.on('end',()=>resolve(text));socket.on('error',reject);});
 assert.match(denied,/403 Forbidden/);
 console.log('Local HTTP + actual WebSocket/JVM + browser transport: physical shots, fresh reconnect, manual stop and origin isolation passed.');
}finally{child.kill();}
