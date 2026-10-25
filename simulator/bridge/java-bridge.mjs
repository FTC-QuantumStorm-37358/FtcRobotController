import {createInterface} from 'node:readline';
import {acceptWebSocket} from './websocket.mjs';
import {compileJava,launchJava} from './java-runtime.mjs';

export function attachJavaBridge(server){
 let status={ready:false,message:'Compiling shared Java controller…'},active=null;
 const ready=compileJava().then(()=>status={ready:true,message:'Java controller ready'}).catch(e=>status={ready:false,message:e.message});
 server.on('upgrade',(req,socket,head)=>{
  if(req.url!=='/robot'){socket.end('HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n');return;}
  let child,lines,timeout,pending=false,disposed=false;
  function dispose(){if(disposed)return;disposed=true;clearTimeout(timeout);lines?.close();child?.stdin.end();child?.kill();if(active===ws)active=null;}
  const ws=acceptWebSocket(req,socket,head,{onClose:dispose,onMessage:line=>{
   if(!child||pending)throw new Error('Java not ready or request already pending');
   const m=JSON.parse(line);if(!['start','step','stop'].includes(m.type))throw new Error('Unsupported command');
   pending=true;clearTimeout(timeout);timeout=setTimeout(()=>{ws.send({type:'error',message:'Java response timed out'});ws.close();},2000);
   if(!child.stdin.write(line+'\n'))throw new Error('Java input overloaded');
  }});
  if(!ws)return;
  if(active){ws.send({type:'error',message:'Autonomous is already connected in another tab.'});ws.close();return;}active=ws;
  ready.then(()=>{
   if(disposed)return;if(!status.ready){ws.send({type:'error',message:status.message});ws.close();return;}
   child=launchJava();lines=createInterface({input:child.stdout});
   timeout=setTimeout(()=>{ws.send({type:'error',message:'Java startup timed out'});ws.close();},5000);
   lines.on('line',line=>{clearTimeout(timeout);pending=false;try{ws.send(JSON.parse(line));}catch{ws.send({type:'error',message:'Invalid Java response'});ws.close();}});
   child.stderr.on('data',()=>{});
   child.on('error',()=>{ws.send({type:'error',message:'Could not start Java'});ws.close();});
   child.on('exit',()=>{if(!disposed){ws.send({type:'error',message:'Java controller stopped'});ws.close();}});
  });
 });
 server.on('close',()=>active?.close());
 return {status:()=>status,ready};
}
