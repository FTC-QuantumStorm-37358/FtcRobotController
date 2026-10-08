import {createHash} from 'node:crypto';
import {TextDecoder} from 'node:util';

const limit=131072;
const decoder=new TextDecoder('utf-8',{fatal:true});
function frame(opcode,payload){
 const bytes=Buffer.isBuffer(payload)?payload:Buffer.from(payload);let header;
 if(bytes.length<126){header=Buffer.from([0x80|opcode,bytes.length]);}
 else if(bytes.length<65536){header=Buffer.alloc(4);header[0]=0x80|opcode;header[1]=126;header.writeUInt16BE(bytes.length,2);}
 else{header=Buffer.alloc(10);header[0]=0x80|opcode;header[1]=127;header.writeBigUInt64BE(BigInt(bytes.length),2);}
 return Buffer.concat([header,bytes]);
}
/** Narrow RFC 6455 text transport; bounded masked input, fragmentation and ping support. */
export function acceptWebSocket(req,socket,head,{onMessage,onClose}){
 const key=req.headers['sec-websocket-key'];
 let originOK=!req.headers.origin;try{originOK||=new URL(req.headers.origin).host===req.headers.host&&new URL(req.headers.origin).protocol==='http:';}catch{}
 if(!originOK||req.headers['sec-websocket-version']!=='13'||typeof key!=='string'||!/^[A-Za-z0-9+/]{22}==$/.test(key)||req.headers.upgrade?.toLowerCase()!=='websocket'){
  socket.end('HTTP/1.1 403 Forbidden\r\nConnection: close\r\n\r\n');return null;
 }
 const accept=createHash('sha1').update(key+'258EAFA5-E914-47DA-95CA-C5AB0DC85B11').digest('base64');
 socket.write('HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: '+accept+'\r\n\r\n');
 let buffer=Buffer.alloc(0),parts=[],total=0,fragmented=false,closed=false;
 const close=()=>{if(closed)return;closed=true;onClose?.();};
 const api={send(value){if(!closed){const b=frame(1,JSON.stringify(value));if(socket.writableLength+b.length>limit*2){socket.destroy();return;}socket.write(b);}},close(){if(!closed){socket.end(frame(8,Buffer.from([3,232])));close();}}};
 function data(chunk){
  buffer=Buffer.concat([buffer,chunk]);if(buffer.length>limit+14){socket.destroy();return;}
  while(buffer.length>=2){
   const first=buffer[0],second=buffer[1],fin=!!(first&128),opcode=first&15;let size=second&127,offset=2;
   if(first&0x70||!(second&128)){socket.destroy();return;}
   if(size===126){if(buffer.length<4)return;size=buffer.readUInt16BE(2);offset=4;}
   else if(size===127){if(buffer.length<10)return;const big=buffer.readBigUInt64BE(2);if(big>BigInt(limit)){socket.destroy();return;}size=Number(big);offset=10;}
   if(size>limit||opcode>=8&&(!fin||size>125)){socket.destroy();return;}
   if(buffer.length<offset+4+size)return;
   const mask=buffer.subarray(offset,offset+4),body=Buffer.from(buffer.subarray(offset+4,offset+4+size));for(let i=0;i<size;i++)body[i]^=mask[i%4];buffer=buffer.subarray(offset+4+size);
   if(opcode===8){api.close();return;}if(opcode===9){socket.write(frame(10,body));continue;}if(opcode===10)continue;
   if(opcode!==0&&opcode!==1||opcode===0&&!fragmented||opcode===1&&fragmented){socket.destroy();return;}
   parts.push(body);total+=size;if(total>limit){socket.destroy();return;}fragmented=!fin;
   if(fin){const bytes=Buffer.concat(parts);parts=[];total=0;try{onMessage(decoder.decode(bytes));}catch{socket.destroy();return;}}
  }
 }
 socket.on('data',data);socket.on('close',close);socket.on('error',close);if(head.length)queueMicrotask(()=>data(head));return api;
}
