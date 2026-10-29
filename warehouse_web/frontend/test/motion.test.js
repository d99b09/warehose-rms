import assert from 'node:assert/strict';
import test from 'node:test';
import { RobotMotion } from '../motion.js';

const robot = overrides => ({ robot_id: 'robot_01', x: 0, y: 0, theta: 0,
  stale: false, odom_age: 0, ...overrides });

test('first pose is immediate; subsequent poses interpolate without extrapolation', () => {
  const motion = new RobotMotion();
  const start = robot();
  motion.update([start], 1000, 2);
  assert.equal(motion.pose(start, 1000).x, 0);
  assert.equal(motion.isAnimating(1000), false);
  const end = robot({ x: 1, y: 2 });
  motion.update([end], 1100, 2);
  assert.deepEqual(motion.pose(end, 1150), { x: 0.5, y: 1, theta: 0 });
  assert.equal(motion.isAnimating(1150), true);
  assert.deepEqual(motion.pose(end, 1500), { x: 1, y: 2, theta: 0 });
  assert.equal(motion.isAnimating(1500), false);
});

test('rotation crosses the angle boundary along the shortest arc', () => {
  const motion = new RobotMotion();
  motion.update([robot({ theta: Math.PI - 0.1 })], 1000, 2);
  const end = robot({ theta: -Math.PI + 0.1 });
  motion.update([end], 1100, 2);
  assert.ok(Math.abs(motion.pose(end, 1150).theta - Math.PI) < 1e-10);
});

test('a new sample during animation starts from the displayed position', () => {
  const motion = new RobotMotion();
  motion.update([robot()], 1000, 2);
  motion.update([robot({ x: 1 })], 1100, 2);
  const next = robot({ x: 2 });
  motion.update([next], 1150, 2);
  assert.equal(motion.pose(next, 1150).x, 0.5);
  assert.equal(motion.pose(next, 1175).x, 1.25);
});

test('duplicate poses do not restart the animation', () => {
  const motion = new RobotMotion();
  motion.update([robot()], 1000, 2);
  const end = robot({ x: 1 });
  motion.update([end], 1100, 2);
  motion.update([end], 1150, 2);
  assert.equal(motion.pose(end, 1150).x, 0.5);
  assert.equal(motion.isAnimating(1200), false);
});

test('stale telemetry and recovery after a gap do not animate obsolete motion', () => {
  const motion = new RobotMotion();
  motion.update([robot()], 1000, 2);
  const stale = robot({ x: 1, stale: true, odom_age: 3 });
  motion.update([stale], 1100, 2);
  assert.equal(motion.pose(stale, 1100).x, 1);
  assert.equal(motion.isAnimating(1100), false);
  motion.update([robot({ x: 2 })], 1200, 2);
  assert.equal(motion.isAnimating(1200), false);
  motion.update([robot({ x: 3 })], 4000, 2);
  assert.equal(motion.isAnimating(4000), false);
});

test('removed robots and missing coordinates discard old poses', () => {
  const motion = new RobotMotion();
  motion.update([robot()], 1000, 2);
  motion.update([robot({ x: null })], 1100, 2);
  assert.equal(motion.poses.size, 0);
  motion.update([robot()], 1200, 2);
  motion.update([], 1300, 2);
  assert.equal(motion.poses.size, 0);
});
