(() => {
  const canvas = document.querySelector('#view');
  const ctx = canvas.getContext('2d');
  const fileInput = document.querySelector('#file');
  const scrub = document.querySelector('#scrub');
  const play = document.querySelector('#play');
  const time = document.querySelector('#time');
  const empty = document.querySelector('#empty');
  let replay = null, frame = 0, playing = false, last = 0, scale = 18;
  const palette = ['#8ee5c0', '#75d9d4', '#9fc4fa', '#dbc1ff', '#f0c987'];
  const project = (x,y,z,cx,cy) => [cx+(x-z)*0.866*scale, cy+(x+z)*0.25*scale-y*scale];
  const resize = () => { const r=canvas.getBoundingClientRect(), d=window.devicePixelRatio||1; canvas.width=Math.round(r.width*d);canvas.height=Math.round(r.height*d);ctx.setTransform(d,0,0,d,0,0);draw(); };
  function drawBox(meta, pose, cx, cy, color) {
    const [x,y,z,qx,qy,qz,qw]=pose, [hx,hy,hz]=meta.size;
    const c=[[-hx,-hy,-hz],[hx,-hy,-hz],[hx,-hy,hz],[-hx,-hy,hz],[-hx,hy,-hz],[hx,hy,-hz],[hx,hy,hz],[-hx,hy,hz]];
    const rotate=(v)=>{const [vx,vy,vz]=v;const tx=2*(qy*vz-qz*vy),ty=2*(qz*vx-qx*vz),tz=2*(qx*vy-qy*vx);return [vx+qw*tx+(qy*tz-qz*ty),vy+qw*ty+(qz*tx-qx*tz),vz+qw*tz+(qx*ty-qy*tx)];};
    const p=c.map(v=>{const r=rotate(v);return project(x+r[0],y+r[1],z+r[2],cx,cy)});
    const faces=[[0,1,2,3,'#334e47'],[0,4,5,1,'#426b5d'],[1,5,6,2,'#52826d'],[2,6,7,3,'#75b893'],[3,7,4,0,'#426b5d'],[4,7,6,5,'#83cba0']];
    faces.forEach(f=>{ctx.beginPath();ctx.moveTo(...p[f[0]]);for(let i=1;i<4;i++)ctx.lineTo(...p[f[i]]);ctx.closePath();ctx.fillStyle=color||f[4];ctx.globalAlpha=meta.static?0.38:0.92;ctx.fill();ctx.strokeStyle=meta.static?'#668078':'#b9f0d4';ctx.globalAlpha=meta.static?0.35:0.32;ctx.lineWidth=0.7;ctx.stroke();});ctx.globalAlpha=1;
  }
  function drawSphere(meta, pose, cx, cy, color) {
    const [x,y,z]=pose,[rx,ry]=project(x,y,z,cx,cy),r=Math.max(2,meta.size[0]*scale*0.88);
    const grad=ctx.createRadialGradient(rx-r*.35,ry-r*.45,r*.08,rx,ry,r);grad.addColorStop(0,'#e8fff1');grad.addColorStop(.26,color);grad.addColorStop(1,'#24483d');
    ctx.beginPath();ctx.ellipse(rx,ry,r,r*.72,0,0,Math.PI*2);ctx.fillStyle=grad;ctx.globalAlpha=meta.static?.48:.96;ctx.fill();ctx.strokeStyle='#cbf5dc';ctx.globalAlpha=.35;ctx.stroke();ctx.globalAlpha=1;
  }
  function draw(){const w=canvas.clientWidth,h=canvas.clientHeight;if(!w||!h)return;ctx.clearRect(0,0,w,h);if(!replay)return;
    const cx=w*.5,cy=h*.66;const floor=project(0,-.51,0,cx,cy),fw=82*scale,fh=24*scale;
    ctx.beginPath();ctx.moveTo(floor[0],floor[1]-fh*.45);ctx.lineTo(floor[0]+fw*.5,floor[1]);ctx.lineTo(floor[0],floor[1]+fh*.45);ctx.lineTo(floor[0]-fw*.5,floor[1]);ctx.closePath();ctx.fillStyle='#1a2525';ctx.fill();ctx.strokeStyle='#344442';ctx.lineWidth=1;ctx.stroke();
    const f=replay.frames[frame]||replay.frames[0];const order=replay.bodies.map((b,i)=>({b,p:f.poses[i],i})).sort((a,b)=>(a.p[0]+a.p[1]+a.p[2])-(b.p[0]+b.p[1]+b.p[2]));
    for(const item of order){const color=palette[item.i%palette.length];if(item.b.type==='sphere')drawSphere(item.b,item.p,cx,cy,color);else drawBox(item.b,item.p,cx,cy,item.b.static?'#466158':color);}
  }
  function setFrame(value){frame=Math.max(0,Math.min(replay.frames.length-1,value));scrub.value=String(frame);time.textContent=`${replay.frames[frame].t.toFixed(2)}s`;draw();}
  function load(data){if(!data||data.version!==1||!Array.isArray(data.bodies)||!Array.isArray(data.frames)||!data.frames.length)throw new Error('Unsupported or empty QuantumBody replay');replay=data;frame=0;playing=false;play.textContent='▶';scrub.max=String(data.frames.length-1);scrub.disabled=false;play.disabled=false;empty.classList.add('hidden');document.querySelector('#scene-name').textContent=data.scene||'Replay';document.querySelector('#body-count').textContent=data.bodies.length.toLocaleString();document.querySelector('#frame-count').textContent=data.frames.length.toLocaleString();document.querySelector('#fixed-step').textContent=`${(data.dt*1000).toFixed(2)} ms`;document.querySelector('#scene-kind').textContent=(data.scene||'—').replaceAll('-',' ').toUpperCase();const extent=Math.max(...data.bodies.map(b=>Math.max(...b.size)),10);scale=Math.min(27,Math.max(5,Math.min(canvas.clientWidth/(extent*3.5),canvas.clientHeight/(extent*1.8))));setFrame(0);}
  fileInput.addEventListener('change',async e=>{const f=e.target.files[0];if(!f)return;try{load(JSON.parse(await f.text()));}catch(err){alert(`Could not load replay: ${err.message}`);}});
  scrub.addEventListener('input',()=>setFrame(Number(scrub.value)));
  play.addEventListener('click',()=>{playing=!playing;play.textContent=playing?'Ⅱ':'▶';last=0;});
  function animate(ts){if(playing&&replay){if(!last)last=ts;const interval=Math.max(12,(replay.dt*1000*(replay.frames[1]?.t/replay.dt||4)));if(ts-last>=interval){setFrame(frame+1);last=ts;if(frame===replay.frames.length-1){playing=false;play.textContent='▶';}}}requestAnimationFrame(animate);}
  window.addEventListener('resize',resize);resize();requestAnimationFrame(animate);
  const params=new URLSearchParams(location.search);if(params.has('file'))fetch(params.get('file')).then(r=>{if(!r.ok)throw new Error('Replay file not found');return r.json();}).then(load).catch(()=>{});
})();
