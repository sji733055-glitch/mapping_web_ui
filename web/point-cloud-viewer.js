(function () {
  "use strict";

  const vertexShaderSource = `
    attribute vec3 aPosition;
    uniform mat4 uMvp;
    uniform float uPointSize;
    uniform vec2 uHeightRange;
    varying float vHeight;
    void main() {
      gl_Position = uMvp * vec4(aPosition, 1.0);
      gl_PointSize = uPointSize;
      vHeight = clamp((aPosition.z - uHeightRange.x) / max(0.001, uHeightRange.y - uHeightRange.x), 0.0, 1.0);
    }
  `;

  const fragmentShaderSource = `
    precision mediump float;
    varying float vHeight;
    vec3 palette(float t) {
      vec3 low = vec3(0.20, 0.43, 0.74);
      vec3 mid = vec3(0.25, 0.88, 0.79);
      vec3 high = vec3(1.0, 0.52, 0.29);
      return t < 0.58 ? mix(low, mid, t / 0.58) : mix(mid, high, (t - 0.58) / 0.42);
    }
    void main() {
      vec2 c = gl_PointCoord - vec2(0.5);
      if (dot(c, c) > 0.25) discard;
      gl_FragColor = vec4(palette(vHeight), 0.88);
    }
  `;

  const lineVertexSource = `
    attribute vec3 aPosition;
    uniform mat4 uMvp;
    void main() { gl_Position = uMvp * vec4(aPosition, 1.0); }
  `;
  const lineFragmentSource = `
    precision mediump float;
    uniform vec4 uColor;
    void main() { gl_FragColor = uColor; }
  `;

  function compile(gl, type, source) {
    const shader = gl.createShader(type);
    gl.shaderSource(shader, source);
    gl.compileShader(shader);
    if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(shader));
    return shader;
  }

  function program(gl, vertex, fragment) {
    const result = gl.createProgram();
    gl.attachShader(result, compile(gl, gl.VERTEX_SHADER, vertex));
    gl.attachShader(result, compile(gl, gl.FRAGMENT_SHADER, fragment));
    gl.linkProgram(result);
    if (!gl.getProgramParameter(result, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(result));
    return result;
  }

  function perspective(out, fovy, aspect, near, far) {
    const f = 1 / Math.tan(fovy / 2);
    out.fill(0); out[0] = f / aspect; out[5] = f; out[10] = (far + near) / (near - far);
    out[11] = -1; out[14] = (2 * far * near) / (near - far); return out;
  }

  function lookAt(out, eye, target, up) {
    let zx = eye[0] - target[0], zy = eye[1] - target[1], zz = eye[2] - target[2];
    let len = Math.hypot(zx, zy, zz) || 1; zx /= len; zy /= len; zz /= len;
    let xx = up[1] * zz - up[2] * zy, xy = up[2] * zx - up[0] * zz, xz = up[0] * zy - up[1] * zx;
    len = Math.hypot(xx, xy, xz) || 1; xx /= len; xy /= len; xz /= len;
    const yx = zy * xz - zz * xy, yy = zz * xx - zx * xz, yz = zx * xy - zy * xx;
    out[0]=xx; out[1]=yx; out[2]=zx; out[3]=0;
    out[4]=xy; out[5]=yy; out[6]=zy; out[7]=0;
    out[8]=xz; out[9]=yz; out[10]=zz; out[11]=0;
    out[12]=-(xx*eye[0]+xy*eye[1]+xz*eye[2]);
    out[13]=-(yx*eye[0]+yy*eye[1]+yz*eye[2]);
    out[14]=-(zx*eye[0]+zy*eye[1]+zz*eye[2]); out[15]=1; return out;
  }

  function multiply(out, a, b) {
    for (let col=0; col<4; col++) for (let row=0; row<4; row++) {
      out[col*4+row] = a[row]*b[col*4] + a[4+row]*b[col*4+1] + a[8+row]*b[col*4+2] + a[12+row]*b[col*4+3];
    }
    return out;
  }

  class PointCloudViewer {
    constructor(canvas) {
      this.canvas = canvas;
      this.gl = canvas.getContext("webgl", { antialias: true, alpha: false });
      if (!this.gl) throw new Error("浏览器不支持 WebGL");
      this.points = new Float32Array(0);
      this.bounds = null;
      this.yaw = -0.72; this.pitch = 0.78; this.distance = 18; this.target = [0, 0, 0];
      this.drag = null; this.needsRender = true;
      this._setup(); this._bind(); this._resize(); this._loop();
    }

    _setup() {
      const gl = this.gl;
      this.pointProgram = program(gl, vertexShaderSource, fragmentShaderSource);
      this.lineProgram = program(gl, lineVertexSource, lineFragmentSource);
      this.pointBuffer = gl.createBuffer(); this.gridBuffer = gl.createBuffer(); this.axisBuffer = gl.createBuffer();
      const grid = [];
      for (let i=-20; i<=20; i++) { grid.push(-20,i,0, 20,i,0, i,-20,0, i,20,0); }
      gl.bindBuffer(gl.ARRAY_BUFFER, this.gridBuffer); gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(grid), gl.STATIC_DRAW); this.gridCount = grid.length/3;
      const axes = [0,0,0, 3,0,0, 0,0,0, 0,3,0, 0,0,0, 0,0,3];
      gl.bindBuffer(gl.ARRAY_BUFFER, this.axisBuffer); gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(axes), gl.STATIC_DRAW);
      gl.clearColor(0.018, 0.037, 0.04, 1); gl.enable(gl.DEPTH_TEST); gl.enable(gl.BLEND); gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
    }

    _bind() {
      this.canvas.addEventListener("pointerdown", (e) => { this.canvas.setPointerCapture(e.pointerId); this.drag = { x:e.clientX, y:e.clientY, button:e.button }; });
      this.canvas.addEventListener("pointermove", (e) => {
        if (!this.drag) return;
        const dx=e.clientX-this.drag.x, dy=e.clientY-this.drag.y; this.drag.x=e.clientX; this.drag.y=e.clientY;
        if (this.drag.button === 2) {
          const scale=this.distance*0.0018; const cy=Math.cos(this.yaw), sy=Math.sin(this.yaw);
          this.target[0] += (-cy*dx + sy*dy)*scale; this.target[1] += (-sy*dx - cy*dy)*scale;
        } else { this.yaw -= dx*0.006; this.pitch=Math.max(0.03,Math.min(1.53,this.pitch+dy*0.006)); }
        this.needsRender=true;
      });
      this.canvas.addEventListener("pointerup", () => { this.drag=null; });
      this.canvas.addEventListener("pointercancel", () => { this.drag=null; });
      this.canvas.addEventListener("contextmenu", (e) => e.preventDefault());
      this.canvas.addEventListener("wheel", (e) => { e.preventDefault(); this.distance=Math.max(.4,Math.min(800,this.distance*Math.exp(e.deltaY*.001))); this.needsRender=true; }, { passive:false });
      window.addEventListener("resize", () => this._resize());
    }

    _resize() {
      const dpr=Math.min(window.devicePixelRatio||1,2), w=Math.max(1,Math.floor(this.canvas.clientWidth*dpr)), h=Math.max(1,Math.floor(this.canvas.clientHeight*dpr));
      if (this.canvas.width!==w || this.canvas.height!==h) { this.canvas.width=w; this.canvas.height=h; this.gl.viewport(0,0,w,h); this.needsRender=true; }
    }

    setPoints(points) {
      this.points=points;
      const gl=this.gl; gl.bindBuffer(gl.ARRAY_BUFFER,this.pointBuffer); gl.bufferData(gl.ARRAY_BUFFER,points,gl.DYNAMIC_DRAW);
      if (points.length) {
        let minX=Infinity,minY=Infinity,minZ=Infinity,maxX=-Infinity,maxY=-Infinity,maxZ=-Infinity;
        for(let i=0;i<points.length;i+=3){const x=points[i],y=points[i+1],z=points[i+2];minX=Math.min(minX,x);minY=Math.min(minY,y);minZ=Math.min(minZ,z);maxX=Math.max(maxX,x);maxY=Math.max(maxY,y);maxZ=Math.max(maxZ,z);}
        this.bounds={minX,minY,minZ,maxX,maxY,maxZ};
      } else this.bounds=null;
      this.needsRender=true;
    }

    fit() {
      if (!this.bounds) { this.target=[0,0,0]; this.distance=18; }
      else { const b=this.bounds; this.target=[(b.minX+b.maxX)/2,(b.minY+b.maxY)/2,(b.minZ+b.maxZ)/2]; this.distance=Math.max(4,Math.hypot(b.maxX-b.minX,b.maxY-b.minY,b.maxZ-b.minZ)*1.15); }
      this.needsRender=true;
    }

    top() { this.pitch=0.035; this.yaw=-Math.PI/2; this.fit(); this.distance*=1.1; this.needsRender=true; }

    _matrix() {
      const eye=[this.target[0]+this.distance*Math.cos(this.pitch)*Math.cos(this.yaw),this.target[1]+this.distance*Math.cos(this.pitch)*Math.sin(this.yaw),this.target[2]+this.distance*Math.sin(this.pitch)];
      const p=new Float32Array(16),v=new Float32Array(16),m=new Float32Array(16);
      perspective(p,Math.PI/4,this.canvas.width/this.canvas.height,.05,2000); lookAt(v,eye,this.target,[0,0,1]); return multiply(m,p,v);
    }

    _drawLines(mvp) {
      const gl=this.gl,p=this.lineProgram; gl.useProgram(p); gl.uniformMatrix4fv(gl.getUniformLocation(p,"uMvp"),false,mvp);
      const loc=gl.getAttribLocation(p,"aPosition"); gl.enableVertexAttribArray(loc); gl.bindBuffer(gl.ARRAY_BUFFER,this.gridBuffer); gl.vertexAttribPointer(loc,3,gl.FLOAT,false,0,0); gl.uniform4f(gl.getUniformLocation(p,"uColor"),.19,.37,.36,.24); gl.drawArrays(gl.LINES,0,this.gridCount);
      gl.bindBuffer(gl.ARRAY_BUFFER,this.axisBuffer); gl.vertexAttribPointer(loc,3,gl.FLOAT,false,0,0);
      const color=gl.getUniformLocation(p,"uColor"); gl.uniform4f(color,1,.25,.22,.75); gl.drawArrays(gl.LINES,0,2); gl.uniform4f(color,.25,1,.55,.75); gl.drawArrays(gl.LINES,2,2); gl.uniform4f(color,.25,.55,1,.75); gl.drawArrays(gl.LINES,4,2);
    }

    _drawPoints(mvp) {
      if (!this.points.length) return;
      const gl=this.gl,p=this.pointProgram; gl.useProgram(p); gl.uniformMatrix4fv(gl.getUniformLocation(p,"uMvp"),false,mvp);
      gl.uniform1f(gl.getUniformLocation(p,"uPointSize"),Math.max(1.5,Math.min(4,window.devicePixelRatio*1.8)));
      const min=this.bounds?this.bounds.minZ:0,max=this.bounds?this.bounds.maxZ:1; gl.uniform2f(gl.getUniformLocation(p,"uHeightRange"),min,max);
      const loc=gl.getAttribLocation(p,"aPosition"); gl.enableVertexAttribArray(loc); gl.bindBuffer(gl.ARRAY_BUFFER,this.pointBuffer); gl.vertexAttribPointer(loc,3,gl.FLOAT,false,0,0); gl.drawArrays(gl.POINTS,0,this.points.length/3);
    }

    _loop() { if (this.needsRender) { this.needsRender=false; this.gl.clear(this.gl.COLOR_BUFFER_BIT|this.gl.DEPTH_BUFFER_BIT); const m=this._matrix(); this._drawLines(m); this._drawPoints(m); } requestAnimationFrame(()=>this._loop()); }
  }

  window.PointCloudViewer = PointCloudViewer;
})();
