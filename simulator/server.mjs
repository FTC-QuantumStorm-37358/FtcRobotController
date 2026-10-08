import http from 'node:http';
import {readFile,stat} from 'node:fs/promises';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {attachJavaBridge} from './bridge/java-bridge.mjs';
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'dist');
const port=Number(process.env.PORT||8000);
const mime={'.html':'text/html; charset=utf-8','.js':'text/javascript; charset=utf-8','.css':'text/css; charset=utf-8','.json':'application/json','.png':'image/png','.svg':'image/svg+xml','.txt':'text/plain; charset=utf-8'};
const server=http.createServer(async(req,res)=>{
 try{
  const url=new URL(req.url,'http://localhost');
  if(url.pathname==='/api/java-status'){res.writeHead(200,{'Content-Type':'application/json','Cache-Control':'no-store'});return res.end(JSON.stringify(bridge.status()));}
  let target=path.resolve(root,'.'+decodeURIComponent(url.pathname));
  if(target!==root&&!target.startsWith(root+path.sep)){res.writeHead(403);return res.end('Forbidden');}
  if((await stat(target)).isDirectory())target=path.join(target,'index.html');
  const bytes=await readFile(target);
  res.writeHead(200,{'Content-Type':mime[path.extname(target)]||'application/octet-stream','Cache-Control':'no-store'});
  res.end(req.method==='HEAD'?undefined:bytes);
 }catch{res.writeHead(404);res.end('Not found');}
});
const bridge=attachJavaBridge(server);
await bridge.ready;
server.listen(port,'127.0.0.1',()=>console.log(`FTC BIOBUZZ Simulator: http://localhost:${server.address().port} (Ctrl+C to stop)`));
