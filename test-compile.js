import { gearsPreset } from './src/presets/presetScenes.js';
import { compileToMJCF } from './src/utils/mjcf.js';

console.log("Compiling Gears Preset to XML:");
const xml = compileToMJCF(gearsPreset);
console.log(xml);
