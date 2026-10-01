// Build: bundle + minify (esbuild), strip GLSL whitespace, then pack into a self-extracting HTML.
// dist/index.html = packed (deflate-raw + base64, unpacked by DecompressionStream); dist/dev.html = readable.
import fs from 'fs'; import zlib from 'zlib'; import path from 'path';
const esbuild = await import('esbuild').catch(() => import(process.env.ESBUILD || '/home/mark/scripts/thechat/node_modules/esbuild/lib/main.js'));
const here = path.dirname(new URL(import.meta.url).pathname);
const glslMin = {
  name: 'glsl-min', setup(b) {
    b.onLoad({ filter: /glsl\.js$/ }, a => {
      let src = fs.readFileSync(a.path, 'utf8');
      src = src.replace(/`([^`]*)`/g, (m, body) => {
        const lines = body.split('\n').map(l => l.replace(/\/\/(?![^\n]*\$\{).*$/, '').trim()).filter(Boolean);
        let out = '';
        for (const l of lines) {
          if (l.startsWith('#')) out += (out && !out.endsWith('\n') ? '\n' : '') + l + '\n';
          else out += (out && !out.endsWith('\n') ? ' ' : '') + l;
        }
        out = out.replace(/[ \t]*([{}();,=+*/<>!?:&|\[\]])[ \t]*/g, '$1').replace(/[ \t]*-[ \t]*/g, '-');
        return '`' + out + '`';
      });
      return { contents: src, loader: 'js' };
    });
  }
};
const shell = fs.readFileSync(here + '/src/index.html', 'utf8');
const minShell = shell.replace(/\n/g, '').replace(/>\s+</g, '><');
fs.mkdirSync(here + '/dist', { recursive: true });
const dev = await esbuild.build({ entryPoints: [here + '/src/main.js'], bundle: true, write: false, format: 'iife', target: 'es2022' });
fs.writeFileSync(here + '/dist/dev.html', shell.replace('%SCRIPT%', () => '<script>' + dev.outputFiles[0].text + '</script>'));
const min = await esbuild.build({ entryPoints: [here + '/src/main.js'], bundle: true, write: false, format: 'iife', target: 'es2022', minify: true, plugins: [glslMin], legalComments: 'none' });
const js = min.outputFiles[0].text;
const raw = zlib.deflateRawSync(Buffer.from(js), { level: 9, memLevel: 9 });
// Demoscene packing: the raw deflate stream sits in a trailing HTML comment and the page fetches its own bytes
// (no base64 overhead). Falls back to base64 if the stream happens to contain a comment terminator.
let html;
if (!raw.includes('-->') && !raw.includes('--!>')) {
  const loader = `<script>fetch(location.href).then(r=>r.arrayBuffer()).then(b=>new Response(new Blob([new Uint8Array(b,b.byteLength-${raw.length + 3},${raw.length})]).stream().pipeThrough(new DecompressionStream("deflate-raw"))).text()).then(s=>(0,eval)(s))</script>`;
  html = Buffer.concat([Buffer.from(minShell.replace('%SCRIPT%', () => loader) + '<!--'), raw, Buffer.from('-->')]);
} else {
  const loader = `<script>fetch("data:;base64,${raw.toString('base64')}").then(r=>new Response(r.body.pipeThrough(new DecompressionStream("deflate-raw"))).text()).then(s=>(0,eval)(s))</script>`;
  html = Buffer.from(minShell.replace('%SCRIPT%', () => loader));
}
if (html.length > 65536) throw new Error(`Packed demo exceeds 64 KiB: ${html.length} bytes`);
fs.writeFileSync(here + '/dist/index.html', html);
console.log(`js ${js.length} B minified -> packed html ${html.length} B (${(html.length / 1024).toFixed(1)} KiB)`);
