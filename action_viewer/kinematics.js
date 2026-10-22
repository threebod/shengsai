// 示意尺寸（mm）：X 向车右，Y 向上，Z 向车后。
export const armMount = { x: 130, z: 40 };
export const trayCenter = { x: 0, z: 190 };
export const armRotation = turret => -Math.PI / 2 + (turret - 260) * Math.PI / 180;

export function toolPoint(state) {
  const radius = 220 + state.h;
  const angle = (state.turret - 260) * Math.PI / 180;
  return { x: armMount.x + Math.cos(angle) * radius, y: 200 - state.lift,
    z: armMount.z - Math.sin(angle) * radius };
}
