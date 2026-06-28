import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';

let createOpenSCADFn: any = null;
let loadingPromise: Promise<any> | null = null;

/**
 * Dynamically loads the openscad-wasm CDN script and returns the createOpenSCAD loader function.
 */
export async function loadCompiler(): Promise<any> {
  if (createOpenSCADFn) return createOpenSCADFn;
  if (loadingPromise) return loadingPromise;

  loadingPromise = (async () => {
    try {
      const cdnUrl = 'https://cdn.jsdelivr.net/npm/openscad-wasm@0.0.4/openscad.js';
      
      // Load the ES module dynamically in the browser, ignoring build-time parsing in Vite
      const module = await import(/* @vite-ignore */ cdnUrl);
      createOpenSCADFn = module.createOpenSCAD;
      if (!createOpenSCADFn) {
        throw new Error('createOpenSCAD export not found in openscad-wasm module.');
      }
      return createOpenSCADFn;
    } catch (err) {
      loadingPromise = null; // reset to allow retries
      throw err;
    }
  })();

  return loadingPromise;
}

/**
 * Returns true if the compiler loader function is fetched and ready.
 */
export function isCompilerReady(): boolean {
  return !!createOpenSCADFn;
}

/**
 * Compiles a raw OpenSCAD source code string into 3D mesh vertex/face arrays.
 * Instantiates a fresh WebAssembly instance each time to prevent Emscripten exit status conflicts.
 */
export async function compileSCAD(
  scadCode: string
): Promise<{ vertices: number[]; faces: number[]; renderVertices: number[] }> {
  // Ensure the script loader function is loaded
  const createOpenSCAD = await loadCompiler();
  if (!createOpenSCAD) {
    throw new Error('OpenSCAD compiler failed to initialize.');
  }

  // Instantiate a fresh compiler instance for this compilation
  const compiler = await createOpenSCAD();

  // Render the OpenSCAD code to ASCII STL string format
  const stlText = await compiler.renderToStl(scadCode);
  if (!stlText || stlText.length === 0) {
    throw new Error('Compilation produced empty output.');
  }

  // Parse STL data into a BufferGeometry using STLLoader
  const loader = new STLLoader();
  const geometry = loader.parse(stlText);

  const positionAttr = geometry.attributes.position;
  if (!positionAttr) {
    throw new Error('Parsed STL geometry does not contain position attributes.');
  }

  const rawVerts = positionAttr.array;
  const uniqueVerts: number[] = [];
  const faces: number[] = [];
  const vertMap = new Map<string, number>();

  // Deduplicate vertices and index the face array
  for (let i = 0; i < rawVerts.length; i += 3) {
    const x = rawVerts[i];
    const y = rawVerts[i + 1];
    const z = rawVerts[i + 2];

    const key = `${x.toFixed(5)},${y.toFixed(5)},${z.toFixed(5)}`;
    let idx = vertMap.get(key);
    if (idx === undefined) {
      idx = uniqueVerts.length / 3;
      uniqueVerts.push(x, y, z);
      vertMap.set(key, idx);
    }
    faces.push(idx);
  }

  // Swap Y and Z for MuJoCo's Z-up space representation
  const renderVertices: number[] = [];
  for (let i = 0; i < uniqueVerts.length; i += 3) {
    const x = uniqueVerts[i];
    const y = uniqueVerts[i + 1];
    const z = uniqueVerts[i + 2];
    renderVertices.push(
      Number(x.toFixed(5)),
      Number((-z).toFixed(5)),
      Number(y.toFixed(5))
    );
  }

  return {
    vertices: uniqueVerts,
    faces,
    renderVertices,
  };
}
