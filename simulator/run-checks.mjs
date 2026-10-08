import {readdir} from 'node:fs/promises';
import {spawnSync} from 'node:child_process';
import {fileURLToPath} from 'node:url';
import path from 'node:path';
const root=path.dirname(fileURLToPath(import.meta.url));
for(const name of (await readdir(path.join(root,'checks'))).filter(x=>x.endsWith('.mjs')).sort()){
 console.log(`\n${name}`);
 const r=spawnSync(process.execPath,[path.join(root,'checks',name)],{cwd:root,stdio:'inherit'});
 if(r.status!==0)process.exit(r.status||1);
}
