export const pendulumXML = `
<mujoco model="pendulum">
  <option timestep="0.001" gravity="0 0 -9.81" />
  
  <worldbody>
    <light directional="true" pos="-0.5 0.5 3" dir="0.5 -0.5 -3" diffuse="0.8 0.8 0.8" />
    <geom name="floor" type="plane" size="5 5 0.1" rgba="0.2 0.2 0.2 1" />
    
    <body name="pole" pos="0 0 2">
      <joint name="hinge" type="hinge" axis="0 1 0" pos="0 0 0" damping="0" />
      <geom name="pole_geom" type="capsule" fromto="0 0 0 1 0 0" size="0.05" rgba="0.6 0.6 0.6 1" />
      
      <body name="bob" pos="1 0 0">
        <geom name="bob_geom" type="sphere" size="0.15" rgba="0.2 0.6 1.0 1" />
      </body>
    </body>
  </worldbody>
</mujoco>
`;
