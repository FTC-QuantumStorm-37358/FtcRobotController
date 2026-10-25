// All distances are inches; theta=0 points toward field north (-Z).
export const ROBOT={halfWidth:8.8,halfLength:9.15};
export const START={x:-61.5,z:46,theta:0};
export const obstacles=[
 ...[-24.73,24.73].flatMap(x=>[-15.5,15.5].map(z=>({x,z,hx:1.0,hz:4.9}))),
 {x:-69,z:24,hx:3,hz:3},{x:69,z:-24,hx:3,hz:3},
 {x:-24,z:-69,hx:3,hz:3},{x:24,z:69,hx:3,hz:3}
];
export function overlap(p,b){
 const c=Math.cos(p.theta),s=Math.sin(p.theta),dx=b.x-p.x,dz=b.z-p.z;
 const {halfWidth:a,halfLength:d}=ROBOT;
 // Separating axes for a rotated robot footprint and an axis-aligned obstacle.
 return Math.abs(dx)<a*Math.abs(c)+d*Math.abs(s)+b.hx &&
 Math.abs(dz)<a*Math.abs(s)+d*Math.abs(c)+b.hz &&
 Math.abs(dx*c-dz*s)<a+b.hx*Math.abs(c)+b.hz*Math.abs(s) &&
 Math.abs(dx*s+dz*c)<d+b.hx*Math.abs(s)+b.hz*Math.abs(c);
}
export function allowed(p){
 const c=Math.abs(Math.cos(p.theta)),s=Math.abs(Math.sin(p.theta));
 return Math.abs(p.x)+ROBOT.halfWidth*c+ROBOT.halfLength*s<=71.9 &&
 Math.abs(p.z)+ROBOT.halfWidth*s+ROBOT.halfLength*c<=71.9 && !obstacles.some(b=>overlap(p,b));
}
export function step(p,forward,strafe,turn,dt,precision=false){
 const q={...p},speed=29*(precision?.35:1),normal=Math.max(1,Math.hypot(forward,strafe)),v=forward/normal*speed,lateral=strafe/normal*speed,omega=turn*1.8*(precision?.45:1);
 let blocked=false;
 const rotation={...q,theta:q.theta+omega*dt};
 if(allowed(rotation))q.theta=rotation.theta;else if(turn)blocked=true;
 const drive={...q,x:q.x+(-Math.sin(q.theta)*v+Math.cos(q.theta)*lateral)*dt,z:q.z+(-Math.cos(q.theta)*v-Math.sin(q.theta)*lateral)*dt};
 if(allowed(drive)){q.x=drive.x;q.z=drive.z;}else if(forward||strafe)blocked=true;
 return {...q,blocked,v,lateral,omega};
}
