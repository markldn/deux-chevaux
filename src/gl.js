// Thin WebGL2 helpers: programs with reflected uniforms, VAO meshes with named attributes, render targets.
export let gl;
export function initGL(canvas) {
  gl = canvas.getContext('webgl2', { antialias: false, alpha: false, powerPreference: 'high-performance' });
  if (!gl) throw new Error('WebGL2 is required');
  if (!gl.getExtension('EXT_color_buffer_float')) throw new Error('EXT_color_buffer_float is required');
  gl.getExtension('OES_texture_float_linear');
  return gl;
}
// attribute name -> location (shaders declare `in vec3 aP;` etc.)
export const LOC = { aP: 0, aN: 1, aC: 2, aM: 3, aJ: 4, aW: 5, aK: 6, iA: 7, iB: 8, iC: 9 };
function sh(type, src) {
  const s = gl.createShader(type); gl.shaderSource(s, src); gl.compileShader(s);
  if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) {
    const log = gl.getShaderInfoLog(s), ln = src.split('\n'), m = /0:(\d+)/.exec(log);
    throw new Error('shader: ' + log + (m ? '\n>> ' + ln[+m[1] - 1] : ''));
  }
  return s;
}
export function program(vs, fs) {
  const p = gl.createProgram();
  gl.attachShader(p, sh(gl.VERTEX_SHADER, vs)); gl.attachShader(p, sh(gl.FRAGMENT_SHADER, fs));
  for (const k in LOC) gl.bindAttribLocation(p, LOC[k], k);
  gl.linkProgram(p);
  if (!gl.getProgramParameter(p, gl.LINK_STATUS)) throw new Error('link: ' + gl.getProgramInfoLog(p));
  const u = {}, n = gl.getProgramParameter(p, gl.ACTIVE_UNIFORMS);
  for (let i = 0; i < n; i++) { const a = gl.getActiveUniform(p, i); u[a.name.replace('[0]', '')] = { l: gl.getUniformLocation(p, a.name), t: a.type, s: a.size }; }
  return {
    p, u, use() { gl.useProgram(p); return this; },
    set(name, v) {
      const e = u[name]; if (!e || v === undefined) return this; const l = e.l;
      switch (e.t) {
        case gl.FLOAT: e.s > 1 ? gl.uniform1fv(l, v) : gl.uniform1f(l, v); break;
        case gl.FLOAT_VEC2: gl.uniform2fv(l, v); break;
        case gl.FLOAT_VEC3: gl.uniform3fv(l, v); break;
        case gl.FLOAT_VEC4: gl.uniform4fv(l, v); break;
        case gl.FLOAT_MAT4: gl.uniformMatrix4fv(l, false, v); break;
        case gl.FLOAT_MAT3: gl.uniformMatrix3fv(l, false, v); break;
        default: gl.uniform1i(l, v);
      }
      return this;
    },
    setAll(o) { for (const k in o) this.set(k, o[k]); return this; }
  };
}
// mesh({aP:[data,size], aN:[data,size,dynamic?]...}, indices, instanced attrs [{name,size}])
export function mesh(attrs, I, inst = []) {
  const vao = gl.createVertexArray(); gl.bindVertexArray(vao);
  const bufs = {}; let nv = 0;
  for (const k in attrs) {
    const [data, size, dyn] = attrs[k], b = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, b);
    gl.bufferData(gl.ARRAY_BUFFER, data instanceof Float32Array ? data : new Float32Array(data), dyn ? gl.DYNAMIC_DRAW : gl.STATIC_DRAW);
    gl.enableVertexAttribArray(LOC[k]); gl.vertexAttribPointer(LOC[k], size, gl.FLOAT, false, 0, 0);
    bufs[k] = b; nv = data.length / size;
  }
  for (const { name, size } of inst) {
    const b = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, b); const loc = LOC[name];
    gl.enableVertexAttribArray(loc); gl.vertexAttribPointer(loc, size, gl.FLOAT, false, 0, 0); gl.vertexAttribDivisor(loc, 1);
    bufs[name] = b;
  }
  let count = nv, idx = 0;
  if (I && I.length) {
    const b = gl.createBuffer(); gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, b);
    gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, I instanceof Uint32Array ? I : new Uint32Array(I), gl.STATIC_DRAW); count = I.length; idx = 1;
  }
  gl.bindVertexArray(null);
  return {
    vao, count, nv,
    update(name, data) { gl.bindBuffer(gl.ARRAY_BUFFER, bufs[name]); gl.bufferData(gl.ARRAY_BUFFER, data, gl.DYNAMIC_DRAW); },
    draw(mode = gl.TRIANGLES, instances = 0, first = 0, n = count) {
      gl.bindVertexArray(vao);
      if (idx) instances ? gl.drawElementsInstanced(mode, n, gl.UNSIGNED_INT, first * 4, instances) : gl.drawElements(mode, n, gl.UNSIGNED_INT, first * 4);
      else instances ? gl.drawArraysInstanced(mode, first, n, instances) : gl.drawArrays(mode, first, n);
    }
  };
}
export function tex(w, h, ifmt, fmt, type, filter = gl.LINEAR, mips = false, data = null) {
  const t = gl.createTexture(); gl.bindTexture(gl.TEXTURE_2D, t);
  if (mips) gl.texStorage2D(gl.TEXTURE_2D, Math.floor(Math.log2(Math.max(w, h))) + 1, ifmt, w, h);
  else gl.texImage2D(gl.TEXTURE_2D, 0, ifmt, w, h, 0, fmt, type, data);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, mips ? gl.LINEAR_MIPMAP_LINEAR : filter);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, filter);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE); gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
  return t;
}
export function fbo(colors, depth) {
  const f = gl.createFramebuffer(); gl.bindFramebuffer(gl.FRAMEBUFFER, f);
  colors.forEach((c, i) => c.rb ? gl.framebufferRenderbuffer(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0 + i, gl.RENDERBUFFER, c.rb) : gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0 + i, gl.TEXTURE_2D, c, 0));
  if (depth) depth.rb ? gl.framebufferRenderbuffer(gl.FRAMEBUFFER, gl.DEPTH_ATTACHMENT, gl.RENDERBUFFER, depth.rb) : gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.DEPTH_ATTACHMENT, gl.TEXTURE_2D, depth, 0);
  gl.drawBuffers(colors.length ? colors.map((_, i) => gl.COLOR_ATTACHMENT0 + i) : [gl.NONE]);
  if (!colors.length) gl.readBuffer(gl.NONE);
  const st = gl.checkFramebufferStatus(gl.FRAMEBUFFER);
  if (st !== gl.FRAMEBUFFER_COMPLETE) throw new Error('fbo incomplete ' + st);
  gl.bindFramebuffer(gl.FRAMEBUFFER, null);
  return f;
}
export function rbo(w, h, fmt, samples) {
  const r = gl.createRenderbuffer(); gl.bindRenderbuffer(gl.RENDERBUFFER, r);
  gl.renderbufferStorageMultisample(gl.RENDERBUFFER, samples, fmt, w, h);
  return { rb: r };
}
