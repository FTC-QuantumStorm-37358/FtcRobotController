// View-only transform; no navigation or autonomous decisions.
export function followFrame(pose,aspect=1){
 const x=-Math.sin(pose.theta),z=-Math.cos(pose.theta),distance=90*Math.max(1,1/Math.max(.4,aspect));
 return {position:{x:pose.x-x*distance,y:85,z:pose.z-z*distance},target:{x:pose.x+x*30,y:18,z:pose.z+z*30}};
}
