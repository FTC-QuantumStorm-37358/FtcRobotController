import * as T from './vendor/three.module.js';
export const FIELD={width:144,tile:24,pivot:43.95,frameWidth:49.46,frameDepth:38.95};
const mat=(color,roughness=.6,metalness=0)=>new T.MeshStandardMaterial({color,roughness,metalness});
const metal=mat('#aebfc9',.38,.72),dark=mat('#242e35',.65,.3),black=mat('#151c21',.9),yellow=mat('#efc838',.47,.1);
const red=mat('#e84746',.42,.2),blue=mat('#327de5',.42,.2),green=mat('#75a94c',.47,.15);
function mesh(g,m,parent,x=0,y=0,z=0){const o=new T.Mesh(g,m);o.position.set(x,y,z);o.castShadow=true;o.receiveShadow=true;parent.add(o);return o;}
function box(p,x,y,z,w,h,d,m){return mesh(new T.BoxGeometry(w,h,d),m,p,x,y,z);}
function cylinder(p,x,y,z,r,h,m,axis='y',segments=24){const o=mesh(new T.CylinderGeometry(r,r,h,segments),m,p,x,y,z);if(axis==='x')o.rotation.z=Math.PI/2;if(axis==='z')o.rotation.x=Math.PI/2;return o;}
function bar(p,a,b,r,m){const av=new T.Vector3(...a),bv=new T.Vector3(...b),delta=bv.clone().sub(av);const o=mesh(new T.CylinderGeometry(r,r,delta.length(),8),m,p);o.position.copy(av.add(bv).multiplyScalar(.5));o.quaternion.setFromUnitVectors(new T.Vector3(0,1,0),delta.normalize());return o;}
function canvasTexture(w,h,draw){const c=document.createElement('canvas');c.width=w;c.height=h;draw(c.getContext('2d'),w,h);const t=new T.CanvasTexture(c);t.colorSpace=T.SRGBColorSpace;return t;}
function textTexture(text,color='#dce7e1',bg=null){return canvasTexture(1024,128,(c,w,h)=>{if(bg){c.fillStyle=bg;c.fillRect(0,0,w,h)}c.fillStyle=color;c.font='bold 65px Arial';c.textAlign='center';c.textBaseline='middle';c.fillText(text,w/2,h/2)});}
function sign(p,text,w,h,x,y,z,color,bg){const m=new T.MeshBasicMaterial({map:textTexture(text,color,bg),transparent:!bg,side:T.DoubleSide,depthWrite:!!bg});return mesh(new T.PlaneGeometry(w,h),m,p,x,y,z);}
function groundText(p,text,w,x,z,color){const a=sign(p,text,w,w/8,x,.035,z,color);a.rotation.x=-Math.PI/2;return a;}
const ballGeo=new T.SphereGeometry(1,20,14);
const poreGeo=new T.CircleGeometry(1,7);
const ballMats={pollen:mat('#f2d345',.52),red:mat('#e54443',.48),blue:mat('#327bea',.48)};
export function ball(p,x,y,z,type){const r=type==='pollen'?1.4:1.8;const g=new T.Group();g.position.set(x,y,z);p.add(g);const b=mesh(ballGeo,ballMats[type],g);b.scale.setScalar(r);
 const dots=new T.InstancedMesh(poreGeo,new T.MeshBasicMaterial({color:type==='pollen'?'#94762c':type==='red'?'#932e35':'#254579',side:T.DoubleSide}),22);const tmp=new T.Object3D();for(let i=0;i<22;i++){const yy=1-2*(i+.5)/22,rad=Math.sqrt(1-yy*yy),ang=i*2.399963;const n=new T.Vector3(rad*Math.cos(ang),yy,rad*Math.sin(ang));tmp.position.copy(n).multiplyScalar(r*1.005);tmp.quaternion.setFromUnitVectors(new T.Vector3(0,0,1),n);tmp.scale.setScalar(r*.13);tmp.updateMatrix();dots.setMatrixAt(i,tmp.matrix);}g.add(dots);g.userData.type=type;return g;}
function torus(p,x,y,z,r,t,m){const o=mesh(new T.TorusGeometry(r,t,8,36),m,p,x,y,z);o.rotation.x=Math.PI/2;return o;}
function flower(p,x,z,rot){const g=new T.Group();g.position.set(x,0,z);g.rotation.y=rot;p.add(g);
 torus(g,0,.43,0,1.62,.23,dark);torus(g,0,4.4,0,2.12,.26,dark);torus(g,0,21.5,0,2.22,.3,yellow);
 for(const xx of [-1.65,1.65])for(const zz of [-1.65,1.65]){cylinder(g,xx,12.95,zz,.33,17.1,green);cylinder(g,xx,4.8,zz,.4,.7,dark);cylinder(g,xx,21.2,zz,.4,.6,yellow);}
 box(g,0,2.15,2.15,.7,3.5,.6,metal);box(g,0,22.25,2.15,4.5,1.25,.15,mat('#7e5fa0'));
 for(let i=0;i<4;i++)ball(g,0,1.83+i*2.8,0,'pollen');
}
function cell(p,color,flip=false){const g=new T.Group();p.add(g);const rim=color==='red'?red:blue;const vertices=[[-10,0],[-10,10],[0,14],[10,10],[10,0]];
 const panel=new T.MeshStandardMaterial({color:'#d9e6e9',transparent:true,opacity:.35,roughness:.35,metalness:.12,side:T.DoubleSide,depthWrite:false});
 const shape=new T.Shape();vertices.forEach(([x,y],i)=>i?shape.lineTo(x,y):shape.moveTo(x,y));shape.closePath();mesh(new T.ShapeGeometry(shape),panel,g,0,0,-6);
 for(let i=0;i<5;i++){const a=vertices[i],b=vertices[(i+1)%5];const pos=new Float32Array([a[0],a[1],-6,b[0],b[1],-6,b[0],b[1],6,a[0],a[1],6]);const geo=new T.BufferGeometry();geo.setAttribute('position',new T.BufferAttribute(pos,3));geo.setIndex([0,1,2,0,2,3]);geo.computeVertexNormals();mesh(geo,panel,g);
 for(const z of [-6,6])bar(g,[a[0],a[1],z],[b[0],b[1],z],.26,rim);bar(g,[a[0],a[1],-6],[a[0],a[1],6],.14,metal);}
 // Fiducial-style markings indicate the tag-cluster locations (not usable AprilTags).
 const tagtex=canvasTexture(256,64,(c)=>{c.fillStyle='#f4f5f2';c.fillRect(0,0,256,64);for(let n=0;n<4;n++){c.fillStyle='#111';c.fillRect(n*64+5,5,54,54);for(let i=0;i<5;i++)for(let j=0;j<5;j++)if((i*7+j*3+n)%3===0){c.fillStyle='#fff';c.fillRect(n*64+12+i*8,12+j*8,7,7)}}});
 const tag=mesh(new T.PlaneGeometry(12,3),new T.MeshBasicMaterial({map:tagtex,side:T.DoubleSide}),g,0,-.08,0);tag.rotation.x=Math.PI/2;
 if(flip)g.rotation.x=Math.PI;return g;
}
export function buildField(){const root=new T.Group();root.name='BIOBUZZ V1 field';root.userData.loadingZones=[-1,1].map(s=>({x:s*66.5,z:s*36,hx:5.5,hz:11.5}));const tileTex=canvasTexture(128,128,(c)=>{c.fillStyle='#727d80';c.fillRect(0,0,128,128);let s=18;for(let i=0;i<2800;i++){s=(s*1664525+1013904223)>>>0;const x=s%128;s=(s*1664525+1013904223)>>>0;const y=s%128;c.fillStyle=i%2?'#7b8587':'#687377';c.fillRect(x,y,1,1)}});tileTex.wrapS=tileTex.wrapT=T.RepeatWrapping;tileTex.repeat.set(3,3);
 const tilemat=new T.MeshStandardMaterial({map:tileTex,roughness:1});box(root,0,-1.15,0,147,1.5,147,dark);
 for(let x=0;x<6;x++)for(let z=0;z<6;z++){const t=box(root,-60+x*24,-.295,-60+z*24,23.91,.59,23.91,tilemat);t.name=`Tile ${x+1},${z+1}`;}
 const acrylic=new T.MeshStandardMaterial({color:'#c3dce5',roughness:.3,transparent:true,opacity:.16,depthWrite:false,side:T.DoubleSide});
 for(const s of [-1,1]){box(root,0,6,s*72.2,144,12,.28,acrylic);box(root,s*72.2,6,0,.28,12,144,acrylic);for(const yy of [.8,12.1]){box(root,0,yy,s*72.6,146,.9,1.15,metal);box(root,s*72.6,yy,0,1.15,.9,146,metal);}}
 for(let i=0;i<=6;i++)for(const side of [-1,1]){box(root,-72+i*24,6,side*72.7,.65,12,.85,dark);box(root,side*72.7,6,-72+i*24,.85,12,.65,dark);}
 // Outside alliance areas and inside loading zones, in manual figure 9-2 orientation.
 for(const s of [-1,1]){const m=s<0?red:blue;const zone=new T.MeshStandardMaterial({color:s<0?'#9b4046':'#305e99',roughness:1});box(root,s*100,-.9,0,54,.07,97,zone);
 for(const z of [-48.5,48.5])box(root,s*100,-.82,z,54,.08,1,m);box(root,s*126.5,-.82,0,1,.08,97,m);
 const loadingZone=root.userData.loadingZones[s<0?0:1],lz=loadingZone.z;box(root,loadingZone.x,.012,lz,loadingZone.hx*2,.024,loadingZone.hz*2,new T.MeshBasicMaterial({color:s<0?'#d54e53':'#3b7dde',transparent:true,opacity:.26,depthWrite:false}));box(root,s*61.5,.04,lz,1,.07,23,m);for(const end of [-1,1])box(root,s*66.5,.04,lz+end*11.5,11,.07,1,m);
 const gz=-s*70.8;box(root,s*60.5,.04,gz,23,.07,2,m);groundText(root,'GARDEN',13,s*60.5,gz-s*4.2,s<0?'#ffd4d4':'#cae2ff');
 const label=groundText(root,s<0?'RED ALLIANCE':'BLUE ALLIANCE',41,s*101,0,s<0?'#e8b5b6':'#b2cdf6');label.rotation.z=s*Math.PI/2;
 for(let i=0;i<4;i++)ball(root,s*(70.2-i*2.85),1.4,gz,'pollen');
 for(let i=0;i<5;i++)ball(root,s*(77+(i%2)*3.7),1.0, (Math.floor(i/2)-1)*3.7,s<0?'red':'blue');
 const n=s<0?4:8;for(let i=0;i<n;i++)ball(root,s*70.4,1.4,lz-10+i*2.85,'pollen');
 }
 flower(root,-24,-69,Math.PI);flower(root,69,-24,Math.PI/2);flower(root,24,69,0);flower(root,-69,24,-Math.PI/2);
 const structure=new T.Group();root.add(structure);structure.name='Hive frame';
 for(const x of [-24.73,24.73]){for(const z of [-19.475,19.475]){bar(structure,[x,.5,z],[x,43.95,0],.58,metal);box(structure,x,.4,z,2.2,.8,2.2,dark);cylinder(structure,x,44,0,1,1.2,dark,'x');}box(structure,x,-.18,0,1.7,.2,38.95,metal);}
 bar(structure,[-24.73,43.95,0],[24.73,43.95,0],.62,metal);
 for(const z of [-19.475,19.475])bar(structure,[-24.73,-.2,z],[24.73,-.2,z],.14,metal);
 const hiveGroups=[];
 for(const s of [-1,1]){const hg=new T.Group();hg.position.set(s*12.75,43.95,0);hg.rotation.x=s<0?Math.PI/6:-Math.PI/6;structure.add(hg);hiveGroups.push(hg);const color=s<0?'red':'blue';
 bar(hg,[0,0,-15.45],[0,0,15.45],.4,metal);cylinder(hg,0,0,0,1.3,2.5,dark,'x');
 const up=cell(hg,color);up.name="Upper upright hive cell";up.position.set(0,.4,s<0?-15.45:15.45);if(s<0)up.rotation.y=Math.PI;
 // Both flat cell floors sit above the same support pole, mirrored around the pivot.
 const down=cell(hg,color);down.name="Lower upright hive cell";down.position.set(0,.4,s<0?15.45:-15.45);if(s>0)down.rotation.y=Math.PI;
 for(let i=0;i<3;i++)ball(up,-7.4+i*3.7,1.85,-3.7,color);
 }
 // Reference dimension and discreet floor coordinates.
 bar(root,[-72,-.6,84],[72,-.6,84],.1,mat('#8b9c95'));for(const x of [-72,72])bar(root,[x,-.6,82],[x,-.6,86],.12,metal);groundText(root,'144 IN / 12 FT',32,0,87,'#b8cdc5');
 for(let i=0;i<6;i++){groundText(root,String(i+1),3.5,-60+i*24,-76.5,'#a2b9bd');groundText(root,String.fromCharCode(65+i),3.5,77,-60+i*24,'#a2b9bd');}
 root.userData.hives=hiveGroups;return root;
}
function perforation(w,h){const t=canvasTexture(64,64,(c)=>{c.fillStyle='#bbc7cb';c.fillRect(0,0,64,64);c.clearRect(0,0,0,0);c.globalCompositeOperation='destination-out';c.beginPath();c.arc(32,32,11,0,Math.PI*2);c.fill();c.globalCompositeOperation='source-over';c.strokeStyle='#e8eef0';c.lineWidth=2;c.beginPath();c.arc(32,32,12,0,Math.PI*2);c.stroke()});t.wrapS=t.wrapT=T.RepeatWrapping;t.repeat.set(w/.63,h/.63);return new T.MeshStandardMaterial({map:t,alphaTest:.5,side:T.DoubleSide,metalness:.6,roughness:.42});}
function perfPlate(p,w,h,x,y,z,rx=0,ry=0){const a=mesh(new T.PlaneGeometry(w,h),perforation(w,h),p,x,y,z);a.rotation.set(rx,ry,0);return a;}
function channel(p,x,y,z,length,rot=0){const g=new T.Group();g.position.set(x,y,z);g.rotation.y=rot;p.add(g);perfPlate(g,1.89,length,0,0,0,-Math.PI/2);for(const s of [-1,1])perfPlate(g,length,1.65,s*.945,.825,0,0,Math.PI/2);return g;}
function driveWheel(p,x,y,z){const g=new T.Group();g.position.set(x,y,z);p.add(g);cylinder(g,0,0,0,1.89,.94,black,'x',40);
 const side=Math.sign(x);cylinder(g,side*.49,0,0,1.51,.10,dark,'x',32);cylinder(g,side*.56,0,0,.43,.15,metal,'x',16);
 const ring=mesh(new T.TorusGeometry(1.59,.025,6,40),yellow,g,side*.555,0,0);ring.rotation.y=Math.PI/2;
 for(let i=0;i<24;i++){const a=i*Math.PI/12;const tread=box(g,0,Math.cos(a)*1.86,Math.sin(a)*1.86,1.0,.09,.21,dark);tread.rotation.x=a;}
 for(let i=0;i<6;i++){const a=i*Math.PI/3;bar(g,[side*.56,Math.cos(a)*.48,Math.sin(a)*.48],[side*.56,Math.cos(a)*1.35,Math.sin(a)*1.35],.13,dark);cylinder(g,side*.63,Math.cos(a)*.9,Math.sin(a)*.9,.075,.045,metal,'x',6);}return g;}
export function buildRobot(){const g=new T.Group();g.name='goBILDA six-wheel StarterBot';const wheels=[];
 for(const s of [-1,1]){channel(g,s*6.35,2.25,0,15.118);for(const z of [-6.15,0,6.15])wheels.push(driveWheel(g,s*7.8,z===0?1.89:2.02,z));}
 channel(g,0,2.25,6.3,11.8,Math.PI/2);channel(g,0,2.25,-6.3,11.8,Math.PI/2);
 // Front pickup roller and the horizontal corner rollers visible in the assembly guide.
 const intake=new T.Group();intake.position.set(0,2.1,-7.1);g.add(intake);cylinder(intake,0,0,0,.15,12.8,metal,'x');
 for(let i=0;i<9;i++){const xx=(i-4)*1.24;cylinder(intake,xx,0,0,1.08,1.05,black,'x',28);for(let j=0;j<12;j++){const a=j*Math.PI/6;const lug=box(intake,xx,Math.cos(a)*1.03,Math.sin(a)*1.03,1.08,.14,.18,dark);lug.rotation.x=a;}}
 channel(g,0,3.5,-7.0,12.8,Math.PI/2);cylinder(g,6.6,2.8,-7.1,1.38,.16,metal,'x');cylinder(g,6.73,2.8,-7.1,.35,.2,dark,'x');
 for(const s of [-1,1]){box(g,s*7.6,2.65,-7.9,1.15,1.2,1.7,dark);cylinder(g,s*7.6,1.05,-7.9,1.15,.7,black);cylinder(g,s*7.6,1.45,-7.9,.65,.14,metal);bar(g,[s*6.2,3.5,-7.1],[s*7.6,3.4,-7.9],.2,metal);}
 // Perforated towers and curved polycarbonate shooter hood.
 const shooterStart=g.children.length;
 for(const s of [-1,1]){perfPlate(g,3.1,13.2,s*4.75,10.25,4.3,0,Math.PI/2);bar(g,[s*4.75,3.65,3],[s*4.75,16.85,3],.16,metal);channel(g,s*4.7,7.7,.9,7.4);}
 perfPlate(g,9.2,8.2,0,4.2,.3,-Math.PI/2);
 const pos=[],uv=[],idx=[];for(let i=0;i<=32;i++){const t=i/32;const z=-4.6+9.2*t,y=5.1+11.7*t*t;for(const x of [-4.6,4.6]){pos.push(x,y,z);uv.push(x<0?0:1,t)}}for(let i=0;i<32;i++){const a=i*2;idx.push(a,a+1,a+2,a+1,a+3,a+2)}const hoodgeo=new T.BufferGeometry();hoodgeo.setAttribute('position',new T.Float32BufferAttribute(pos,3));hoodgeo.setAttribute('uv',new T.Float32BufferAttribute(uv,2));hoodgeo.setIndex(idx);hoodgeo.computeVertexNormals();mesh(hoodgeo,perforation(9.2,14.2),g);
 for(const s of [-1,1]){const pts=[];for(let i=0;i<=32;i++){const t=i/32;pts.push(new T.Vector3(s*4.6,5.1+11.7*t*t,-4.6+9.2*t));}mesh(new T.TubeGeometry(new T.CatmullRomCurve3(pts),32,.075,6,false),metal,g);}
 const shooter=cylinder(g,0,9.5,1.65,1.89,1.25,black,'x',36);cylinder(g,0,9.5,1.65,.18,12.4,metal,'x');
 for(const s of [1]){cylinder(g,s*5.9,9.5,1.65,.71,2.3,metal,'x');cylinder(g,s*6.8,9.5,1.65,.75,.9,yellow,'x');cylinder(g,s*7.35,9.5,1.65,.73,.3,black,'x');}
 g.userData.shooterAssembly=g.children.slice(shooterStart);
 for(const s of [-1,1]){cylinder(g,s*4.65,3.35,5.45,.72,3.2,metal,'z');cylinder(g,s*4.65,3.35,4.2,.75,.7,yellow,'z');}
 box(g,3.6,3.6,4.5,3.1,2,3.4,dark);box(g,-2.6,3.15,5.9,3.6,1.8,2.6,black);
 const tag=sign(g,'goBILDA',4.1,.75,3.6,4.62,4.5,'#edce37','#192329');tag.rotation.x=-Math.PI/2;
 for(let i=0;i<4;i++)ball(g,(i%2-.5)*2.85,5.6,-3.2+Math.floor(i/2)*2.85,'pollen');
 g.userData.wheels=wheels;g.userData.intake=intake;g.userData.shooter=shooter;return g;
}
