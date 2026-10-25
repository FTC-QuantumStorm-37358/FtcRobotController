import {readFile,writeFile,stat} from 'node:fs/promises';
import {spawnSync} from 'node:child_process';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
const simulator=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
const arg=process.argv[2];if(!arg){console.error('Usage: npm run link:ftc -- /path/to/FtcRobotController');process.exit(1);}
const ftc=path.resolve(arg),gradle=path.join(ftc,'TeamCode/build.gradle'),local=path.join(ftc,'local.properties');
const marker='// BIOBUZZ shared Java sources';
try{
 if(!/^[^\r\n]+$/.test(simulator))throw new Error('Project path contains a newline.');
 await stat(gradle);
 const ignored=spawnSync('git',['check-ignore','-q','local.properties'],{cwd:ftc});
 if(ignored.status!==0)throw new Error('Add local.properties to the FTC repository .gitignore before linking; it contains machine-specific paths.');
 let properties='';try{properties=await readFile(local,'utf8');}catch(e){if(e.code!=='ENOENT')throw e;}
 const escaped=simulator.replace(/\\/g,'\\\\').replace(/([ :=#!])/g,'\\$1').replace(/[^\x20-\x7e]/g,c=>'\\u'+c.charCodeAt(0).toString(16).padStart(4,'0'));
 properties=properties.replace(/^biobuzz\.root=.*(?:\r?\n|$)/gm,'');
 if(properties&&!properties.endsWith('\n'))properties+='\n';properties+='biobuzz.root='+escaped+'\n';
 let build=await readFile(gradle,'utf8');
 if(!build.includes(marker))build+='\n'+marker+`\ndef biobuzzSettings = new Properties()\ndef biobuzzLocal = rootProject.file('local.properties')\nif (biobuzzLocal.exists()) biobuzzLocal.withInputStream { biobuzzSettings.load(it) }\ndef biobuzzPath = project.findProperty('biobuzzRoot') ?: biobuzzSettings.getProperty('biobuzz.root')\nif (biobuzzPath) {\n    ext.biobuzzRoot = biobuzzPath\n    apply from: new File(biobuzzPath, 'robot/ftc/shared-sources.gradle')\n}\n`;
 await writeFile(local,properties);await writeFile(gradle,build);
 console.log('Linked shared Java sources. No Java files copied. Review TeamCode/build.gradle, configure FtcHardwareConfig, then build in Android Studio.');
}catch(e){console.error(e.message);process.exitCode=1;}
