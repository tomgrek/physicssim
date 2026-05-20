import { gearsPreset } from '../src/presets/presetScenes.js';
import { compileToMJCF } from '../src/utils/mjcf.js';

const xml = compileToMJCF(gearsPreset);
console.log(xml);
