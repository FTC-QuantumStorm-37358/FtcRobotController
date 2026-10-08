import {spawn} from 'node:child_process';
import {existsSync} from 'node:fs';
import {readdir,mkdir} from 'node:fs/promises';
import path from 'node:path';
import {fileURLToPath} from 'node:url';

export const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
export const classes=path.join(root,'robot/build/classes');
const androidStudioJava='/Applications/Android Studio.app/Contents/jbr/Contents/Home/bin/java';
export const java=process.env.JAVA_HOME?path.join(process.env.JAVA_HOME,'bin/java'):existsSync(androidStudioJava)?androidStudioJava:'java';
async function sources(dir){const result=[];for(const entry of await readdir(dir,{withFileTypes:true})){const file=path.join(dir,entry.name);if(entry.isDirectory())result.push(...await sources(file));else if(file.endsWith('.java'))result.push(file);}return result.sort();}
export async function compileJava(extraSources=[]){
 await mkdir(classes,{recursive:true});
 const files=[...await sources(path.join(root,'robot/shared/src/main/java')),...await sources(path.join(root,'robot/desktop/src/main/java')),...extraSources];
 // The JDK compiler module also works when javac is not on PATH.
 return new Promise((resolve,reject)=>{
  const child=spawn(java,['com.sun.tools.javac.Main','--release','8','-encoding','UTF-8','-d',classes,...files],{cwd:root,stdio:['ignore','pipe','pipe']});let diagnostic='';
  child.stdout.on('data',d=>diagnostic+=d);child.stderr.on('data',d=>diagnostic+=d);
  child.on('error',()=>reject(new Error('Java unavailable. Install JDK 17 or newer and set JAVA_HOME if needed.')));
  child.on('exit',code=>code===0?resolve({ready:true}):reject(new Error('Java compile failed: '+diagnostic.trim())));
 });
}
export function launchJava(){return spawn(java,['-cp',classes,'org.firstinspires.ftc.teamcode.biobuzz.desktop.SimulatorMain'],{cwd:root,stdio:['pipe','pipe','pipe']});}
if(process.argv[1]===fileURLToPath(import.meta.url)){
 try{await compileJava();console.log('Shared Java controller compiled for Java 8 / FTC Android.');}catch(e){console.error(e.message);process.exitCode=1;}
}
